#ifndef MASSTREELHM_LHM_NAMESPACE_HH
#define MASSTREELHM_LHM_NAMESPACE_HH

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "kvthread.hh"
#include "directory_meta.hh"
#include "masstree.hh"
#include "masstree_get.hh"
#include "masstree_insert.hh"
#include "masstree_remove.hh"
#include "masstree_scan.hh"
#include "path_key.hh"
#include "persistent_store.hh"

#include <sys/stat.h>

namespace MasstreeLHM {

// entry_kind 描述同一张命名空间表中的最小记录类型。
// 当前阶段只区分目录和普通文件，为后续 rename 和 inode
// 外存指针预留扩展空间。
enum class entry_kind : uint8_t {
    invalid = 0,
    directory = 1,
    file = 2
};

struct namespace_entry {
    entry_kind kind;
    inode_ref ref;
    uint8_t name_length;
    char name[kMaxEntryNameBytes + 1];
};

// readdir_record 描述一次最小目录扫描结果。
// 当前阶段已经可以同时返回：
// 1. 完整哈希路径
// 2. 当前目录下的直接孩子哈希
// 3. 真实目录项名字
// 4. 对应 entry
struct readdir_record {
    std::vector<uint64_t> full_hashes;
    uint64_t child_component_hash;
    std::string child_name;
    namespace_entry entry;
};

struct subtree_record {
    std::vector<uint64_t> full_hashes;
    namespace_entry entry;
};

struct directory_root_debug_info {
    bool found;
    bool is_leaf;
    bool has_meta;
    uint32_t height;
    int size;
    bool child0_is_leaf;
    bool child1_exists;
    bool child1_is_leaf;
    int child0_size;
    int child1_size;
};

inline std::string entry_name(const namespace_entry& entry) {
    return std::string(entry.name, entry.name + entry.name_length);
}

inline std::string_view entry_name_view(const namespace_entry& entry) {
    return std::string_view(entry.name, entry.name_length);
}

inline bool entry_name_fits(const std::string& name) {
    return name.size() <= kMaxEntryNameBytes;
}

inline namespace_entry make_namespace_entry(entry_kind kind, inode_ref ref,
                                            const std::string& name) {
    namespace_entry entry;
    entry.kind = kind;
    entry.ref = ref;
    entry.name_length = static_cast<uint8_t>(name.size());
    memset(entry.name, 0, sizeof(entry.name));
    memcpy(entry.name, name.data(), name.size());
    return entry;
}

inline bool entry_is_valid(const namespace_entry& entry) {
    return entry.kind != entry_kind::invalid;
}

inline bool entry_is_directory(const namespace_entry& entry) {
    return entry.kind == entry_kind::directory;
}

inline bool entry_is_file(const namespace_entry& entry) {
    return entry.kind == entry_kind::file;
}

inline bool entry_name_equals(const namespace_entry& entry, std::string_view name) {
    const size_t n = static_cast<size_t>(entry.name_length);
    return n == name.size() && (n == 0 || memcmp(entry.name, name.data(), n) == 0);
}

inline const char* entry_kind_name(entry_kind kind) {
    switch (kind) {
    case entry_kind::directory:
        return "directory";
    case entry_kind::file:
        return "file";
    default:
        return "invalid";
    }
}

// 命名空间表的 value 比简单 uint64_t 大得多，尤其在加入固定长度文件名后，
// 如果仍使用默认宽度，leaf 节点尺寸会超过当前线程池分配器允许的上限。
// 因此这里先把宽度调小，保证“最小原型 + 文件名承载”能够正常运行。
struct namespace_table_params : Masstree::nodeparams<7, 7> {
    using value_type = namespace_entry;
    using value_print_type = Masstree::value_print<value_type>;
    using threadinfo_type = ::threadinfo;
};

// LhmNamespace 是第二阶段的薄包装层：
// 负责把“路径字符串 -> PathKey -> Masstree 操作”这条链路收敛成
// 更接近文件系统语义的接口。
class LhmNamespace {
  public:
    using table_type = Masstree::basic_table<namespace_table_params>;
    using unlocked_cursor_type = typename table_type::unlocked_cursor_type;
    using cursor_type = Masstree::tcursor<namespace_table_params>;
    using value_type = namespace_entry;

    void initialize(threadinfo& ti) {
        table_.initialize(ti);
        initialize_persistence_store();
    }

    void destroy(threadinfo& ti) {
        if (persistent_store_.IsOpen()) {
            (void) persistent_store_.Sync();
            persistent_store_.Close();
        }
        root_persistent_ref_ = make_inode_ref(0, 0);
        root_persistent_ref_valid_ = false;
        table_.destroy(ti);
    }

    const std::string& persistence_backing_file() const {
        return persistent_store_.BackingFile();
    }

    // fast 模式用于先打通持久化主链路：
    // - 默认开启；
    // - 弱化 create/delete/rename 的存在性与类型检查；
    // - 仅用于原型阶段压缩流程开销。
    void set_fast_mode(bool enabled) {
        fast_mode_ = enabled;
    }

    bool fast_mode() const {
        return fast_mode_;
    }

    // L0 inode 缓冲开关：
    // - true: 启用 L0+L1 两级缓存（更适合冷启动批量写）
    // - false: 只启用 L1（更适合常规稳态测试）
    bool set_l0_cache_enabled(bool enabled) {
        l0_cache_enabled_ = enabled;
        return persistent_store_.set_l0_cache_enabled(enabled);
    }

    bool l0_cache_enabled() const {
        return l0_cache_enabled_;
    }

    // lookup_entry 返回任意类型的命名空间项，根目录 "/" 被视为隐式存在。
    bool lookup_entry(const std::string& path, value_type& out, threadinfo& ti) const {
        ParsedPath parsed = PathKey::parse_absolute_path(path);
        return lookup_entry_from_parsed(parsed, out, ti);
    }

    // lookup_file 只在目标路径存在且类型为普通文件时返回成功。
    bool lookup_file(const std::string& path, value_type& out, threadinfo& ti) const {
        if (!lookup_entry(path, out, ti)) {
            return false;
        }
        return entry_is_file(out);
    }

    // lookup_directory 只在目标路径存在且类型为目录时返回成功。
    bool lookup_directory(const std::string& path, value_type& out, threadinfo& ti) const {
        if (!lookup_entry(path, out, ti)) {
            return false;
        }
        return entry_is_directory(out);
    }

    // mkdir（推荐接口）：
    // 调用方只提供路径，inode 引用由命名空间层自动分配。
    bool mkdir(const std::string& path, threadinfo& ti) {
        return mkdir(path, make_inode_ref(0, 0), ti);
    }

    // mkdir 兼容接口：
    // 保留显式 ref 传参，主要用于旧测试/回放场景。
    bool mkdir(const std::string& path, inode_ref ref, threadinfo& ti) {
        ParsedPath parsed = PathKey::parse_absolute_path(path);
        if (parsed.normalized_path == "/" || parsed.components.empty()) {
            return false;
        }
        if (!entry_name_fits(parsed.components.back())) {
            return false;
        }

        node_type* parent_root = nullptr;
        inode_ref parent_ref = make_inode_ref(0, 0);
        if (!resolve_parent_creation_context(parsed, parent_root, parent_ref, ti)) {
            return false;
        }

        inode_ref persistent_ref = make_inode_ref(0, 0);
        if (!allocate_inode_ref_for_create(ref, persistent_ref)) {
            return false;
        }
        if (!create_directory_on_parent_root(parsed, parent_root, persistent_ref, ti)) {
            return false;
        }
        return persist_create_success(parsed, entry_kind::directory, persistent_ref, parent_ref);
    }

    // creat_file（推荐接口）：
    // 调用方只提供路径，inode 引用由命名空间层自动分配。
    bool creat_file(const std::string& path, threadinfo& ti) {
        return creat_file(path, make_inode_ref(0, 0), ti);
    }

    // creat_file 兼容接口：
    // 保留显式 ref 传参，主要用于旧测试/回放场景。
    bool creat_file(const std::string& path, inode_ref ref, threadinfo& ti) {
        ParsedPath parsed = PathKey::parse_absolute_path(path);
        if (parsed.normalized_path == "/" || parsed.components.empty()) {
            return false;
        }
        if (!entry_name_fits(parsed.components.back())) {
            return false;
        }

        node_type* parent_root = nullptr;
        inode_ref parent_ref = make_inode_ref(0, 0);
        if (!resolve_parent_creation_context(parsed, parent_root, parent_ref, ti)) {
            return false;
        }

        inode_ref persistent_ref = make_inode_ref(0, 0);
        if (!allocate_inode_ref_for_create(ref, persistent_ref)) {
            return false;
        }
        if (!create_file_on_parent_root(parsed, parent_root, persistent_ref, ti)) {
            return false;
        }
        return persist_create_success(parsed, entry_kind::file, persistent_ref, parent_ref);
    }

    // 仅用于哈希碰撞测试：强制将最后一级路径分量替换成指定哈希值，
    // 用于观察“忽略碰撞模式”下的行为。
    bool creat_file_with_forced_last_hash_for_test(const std::string& path,
                                                   uint64_t forced_hash, threadinfo& ti) {
        return creat_file_with_forced_last_hash_for_test(path, make_inode_ref(0, 0),
                                                         forced_hash, ti);
    }

    // 兼容接口：保留显式 ref 注入能力，供旧测试代码继续使用。
    bool creat_file_with_forced_last_hash_for_test(const std::string& path, inode_ref ref,
                                                   uint64_t forced_hash, threadinfo& ti) {
        ParsedPath parsed = PathKey::parse_absolute_path(path);
        if (parsed.normalized_path == "/" || parsed.components.empty() || parsed.hashes.empty()) {
            return false;
        }
        if (!entry_name_fits(parsed.components.back())) {
            return false;
        }
        parsed.hashes.back() = forced_hash;

        node_type* parent_root = nullptr;
        inode_ref parent_ref = make_inode_ref(0, 0);
        if (!resolve_parent_creation_context(parsed, parent_root, parent_ref, ti)) {
            return false;
        }

        inode_ref persistent_ref = make_inode_ref(0, 0);
        if (!allocate_inode_ref_for_create(ref, persistent_ref)) {
            return false;
        }
        if (!create_file_on_parent_root(parsed, parent_root, persistent_ref, ti)) {
            return false;
        }
        return persist_create_success(parsed, entry_kind::file, persistent_ref, parent_ref);
    }

    bool lookup_file_with_forced_last_hash_for_test(const std::string& path, value_type& out,
                                                    uint64_t forced_hash, threadinfo& ti) const {
        ParsedPath parsed = PathKey::parse_absolute_path(path);
        if (parsed.hashes.empty()) {
            return false;
        }
        parsed.hashes.back() = forced_hash;
        if (!lookup_entry_from_parsed(parsed, out, ti)) {
            return false;
        }
        return entry_is_file(out);
    }

    // readdir 只扫描“当前目录对应 layer root”这一层的孩子项。
    // 扫描顺序按 Masstree 键序从左到右；命中目录 edge 时只读取目录元数据，
    // 不下钻到子目录内部。
    std::vector<readdir_record> readdir(const std::string& path, threadinfo& ti) const {
        ParsedPath parsed = PathKey::parse_absolute_path(path);
        std::vector<readdir_record> results;
        const node_type* directory_root = nullptr;

        if (parsed.normalized_path != "/") {
            if (!locate_directory_root(parsed, directory_root, ti)) {
                return results;
            }
            if (!directory_root->has_directory_meta()) {
                return results;
            }
            const directory_meta* meta = directory_root->directory_meta();
            if (meta == nullptr || !directory_meta_name_equals(*meta, parsed.components.back())) {
                return results;
            }
        } else {
            directory_root = table_.root();
        }

        append_directory_children(directory_root, parsed.hashes, results);
        return results;
    }

    // 当前 rename 是单表原型上的“可工作语义实现”，不是最终 O(1) 版本。
    // 它通过扫描源子树、重建目标路径并逐项重插入/删除来完成：
    // - 普通文件 rename：移动一个 entry
    // - 目录 rename：递归移动整棵子树
    // 后续真正的 O(1) 目录 rename 仍需要依赖 Masstree 内核中的子树入口切换。
    bool rename_path(const std::string& old_path, const std::string& new_path, threadinfo& ti) {
        ParsedPath old_parsed = PathKey::parse_absolute_path(old_path);
        ParsedPath new_parsed = PathKey::parse_absolute_path(new_path);

        if (fast_mode_) {
            value_type moved_entry;
            bool ok = rename_path_fast_from_parsed(old_parsed, new_parsed, ti, &moved_entry);
            if (!ok) {
                return false;
            }
            return persist_rename_success(old_parsed, new_parsed, moved_entry, ti);
        }

        if (old_parsed.normalized_path == "/" || new_parsed.normalized_path == "/") {
            return false;
        }
        if (old_parsed.normalized_path == new_parsed.normalized_path) {
            return true;
        }
        if (is_prefix_path(old_parsed.normalized_path, new_parsed.normalized_path)) {
            return false;
        }

        value_type source_entry;
        if (!lookup_entry_from_parsed(old_parsed, source_entry, ti)) {
            return false;
        }

        if (entry_is_directory(source_entry)) {
            if (!rename_directory_o1_from_parsed(old_parsed, new_parsed, ti)) {
                return false;
            }
            return persist_rename_success(old_parsed, new_parsed, source_entry, ti);
        }

        value_type target_entry;
        if (lookup_entry_from_parsed(new_parsed, target_entry, ti)) {
            return false;
        }

        value_type new_parent;
        if (!lookup_directory(parent_path(new_parsed.normalized_path), new_parent, ti)) {
            return false;
        }

        std::vector<subtree_record> subtree = scan_subtree(old_parsed, ti);
        if (subtree.empty()) {
            return false;
        }

        std::map<std::string, namespace_entry> source_path_map;
        for (const subtree_record& record : subtree) {
            source_path_map.emplace(hash_path_key(record.full_hashes), record.entry);
        }

        std::vector<rename_plan_item> plan;
        plan.reserve(subtree.size());
        for (const subtree_record& record : subtree) {
            std::string source_path = reconstruct_path_under_root(
                old_parsed.normalized_path, old_parsed.hashes, record.full_hashes, source_path_map);
            if (source_path.empty()) {
                return false;
            }

            std::string destination_path = rewrite_destination_path(
                source_path, old_parsed.normalized_path, new_parsed.normalized_path);
            if (destination_path.empty()) {
                return false;
            }

            ParsedPath destination_parsed = PathKey::parse_absolute_path(destination_path);
            value_type existing;
            if (lookup_entry_from_parsed(destination_parsed, existing, ti)) {
                return false;
            }

            rename_plan_item item;
            item.source_path = std::move(source_path);
            item.destination_path = std::move(destination_path);
            item.entry = record.entry;
            item.depth = destination_parsed.hashes.size();
            plan.push_back(std::move(item));
        }

        std::sort(plan.begin(), plan.end(),
                  [](const rename_plan_item& a, const rename_plan_item& b) {
                      if (a.depth != b.depth) {
                          return a.depth < b.depth;
                      }
                      return a.destination_path < b.destination_path;
                  });

        for (const rename_plan_item& item : plan) {
            if (!create_entry(item.destination_path, item.entry.kind, item.entry.ref, ti)) {
                return false;
            }
        }

        std::sort(plan.begin(), plan.end(),
                  [](const rename_plan_item& a, const rename_plan_item& b) {
                      if (a.depth != b.depth) {
                          return a.depth > b.depth;
                      }
                      return a.source_path > b.source_path;
                  });

        for (const rename_plan_item& item : plan) {
            if (!remove_entry(item.source_path, ti)) {
                return false;
            }
        }

        return persist_rename_success(old_parsed, new_parsed, source_entry, ti);
    }

    // 测试辅助：最小删除接口，复用当前命名空间层内部的 remove_entry。
    bool remove_path_for_test(const std::string& path, threadinfo& ti) {
        ParsedPath parsed = PathKey::parse_absolute_path(path);
        value_type removed{};
        if (!remove_entry_from_parsed(parsed, &removed, ti)) {
            return false;
        }
        return persist_delete_success(removed, ti);
    }

    // 测试辅助：读取某个目录当前 root 的结构形态，
    // 用来验证 gc_layer() 是否真的把 root internode 收缩到了 child root。
    bool debug_directory_root_info(const std::string& path,
                                   directory_root_debug_info& out,
                                   threadinfo& ti) const {
        ParsedPath parsed = PathKey::parse_absolute_path(path);
        const node_type* root = nullptr;
        if (parsed.normalized_path == "/") {
            root = table_.root();
        } else if (!locate_directory_root(parsed, root, ti)) {
            out = directory_root_debug_info{false, false, false, 0, 0, false,
                                            false, false, -1, -1};
            return false;
        }

        out.found = true;
        out.is_leaf = root->isleaf();
        out.has_meta = root->has_directory_meta();
        out.child0_is_leaf = false;
        out.child1_exists = false;
        out.child1_is_leaf = false;
        out.child0_size = -1;
        out.child1_size = -1;
        if (root->isleaf()) {
            const typename node_type::leaf_type* lf =
                static_cast<const typename node_type::leaf_type*>(root);
            out.height = 0;
            out.size = lf->size();
        } else {
            const typename node_type::internode_type* in =
                static_cast<const typename node_type::internode_type*>(root);
            out.height = in->height_;
            out.size = in->size();
            out.child0_is_leaf = in->child_[0] ? in->child_[0]->isleaf() : false;
            if (in->child_[0]) {
                out.child0_size = in->child_[0]->isleaf()
                    ? static_cast<const typename node_type::leaf_type*>(in->child_[0])->size()
                    : static_cast<const typename node_type::internode_type*>(in->child_[0])->size();
            }
            if (in->nkeys_ > 0 && in->child_[1]) {
                out.child1_exists = true;
                out.child1_is_leaf = in->child_[1]->isleaf();
                out.child1_size = in->child_[1]->isleaf()
                    ? static_cast<const typename node_type::leaf_type*>(in->child_[1])->size()
                    : static_cast<const typename node_type::internode_type*>(in->child_[1])->size();
            }
        }
        return true;
    }

  private:
    using node_type = typename table_type::node_type;

    struct rename_plan_item {
        std::string source_path;
        std::string destination_path;
        namespace_entry entry;
        size_t depth;
    };

    struct directory_edge_lookup {
        bool found = false;
        node_type* child_root = nullptr;
        value_type entry{};
    };

    struct child_slot_lookup {
        bool found = false;
        bool is_layer = false;
        node_type* child_root = nullptr;
        value_type value{};
    };

    struct readdir_scanner {
        const std::vector<uint64_t>* parent_hashes;
        std::vector<readdir_record>* out;

        template <typename SS, typename K>
        void visit_leaf(const SS&, const K&, threadinfo&) {
        }

        bool visit_value(lcdf::Str key, value_type value, threadinfo&) {
            std::vector<uint64_t> decoded = PathKey::decode(key);
            if (decoded.size() != parent_hashes->size() + 1) {
                return true;
            }
            for (size_t i = 0; i < parent_hashes->size(); ++i) {
                if (decoded[i] != (*parent_hashes)[i]) {
                    return true;
                }
            }

            readdir_record record;
            record.child_component_hash = decoded.back();
            record.child_name = entry_name(value);
            record.full_hashes = decoded;
            record.entry = value;
            out->push_back(std::move(record));
            return true;
        }
    };

    struct subtree_scanner {
        const std::vector<uint64_t>* root_hashes;
        std::vector<subtree_record>* out;

        template <typename SS, typename K>
        void visit_leaf(const SS&, const K&, threadinfo&) {
        }

        bool visit_value(lcdf::Str key, value_type value, threadinfo&) {
            std::vector<uint64_t> decoded = PathKey::decode(key);
            if (decoded.size() < root_hashes->size()) {
                return true;
            }
            for (size_t i = 0; i < root_hashes->size(); ++i) {
                if (decoded[i] != (*root_hashes)[i]) {
                    return true;
                }
            }

            subtree_record record;
            record.full_hashes = std::move(decoded);
            record.entry = value;
            out->push_back(std::move(record));
            return true;
        }
    };

    table_type table_;
    bool fast_mode_ = true;
    bool l0_cache_enabled_ = true;
    PersistentStore persistent_store_;
    bool persistence_available_ = false;
    std::atomic<uint64_t> transient_inode_cursor_{1};
    inode_ref root_persistent_ref_ = make_inode_ref(0, 0);
    bool root_persistent_ref_valid_ = false;

    struct single_component_key {
        char bytes[sizeof(uint64_t)];

        lcdf::Str as_str() const {
            return lcdf::Str(bytes, sizeof(bytes));
        }
    };

    static single_component_key make_single_component_key(uint64_t component_hash) {
        single_component_key key;
        uint64_t be = host_to_net_order(component_hash);
        memcpy(key.bytes, &be, sizeof(be));
        return key;
    }

    static node_type* canonicalize_directory_root(node_type* root) {
        while (true) {
            node_type* next = root->maybe_parent();
            if (next == root) {
                return root;
            }
            root = next;
        }
    }

    static const node_type* canonicalize_directory_root(const node_type* root) {
        while (true) {
            const node_type* next = root->maybe_parent();
            if (next == root) {
                return root;
            }
            root = next;
        }
    }

    bool directory_root_has_children(const node_type* node) const {
        if (node->isleaf()) {
            const typename node_type::leaf_type* leaf =
                static_cast<const typename node_type::leaf_type*>(node);
            return leaf->size() != 0;
        }

        const typename node_type::internode_type* in =
            static_cast<const typename node_type::internode_type*>(node);
        for (int i = 0; i != in->size() + 1; ++i) {
            if (in->child_[i] && directory_root_has_children(in->child_[i])) {
                return true;
            }
        }
        return false;
    }

    static std::string parent_path(const std::string& normalized_path) {
        if (normalized_path == "/") {
            return "/";
        }

        size_t pos = normalized_path.find_last_of('/');
        if (pos == 0) {
            return "/";
        }
        return normalized_path.substr(0, pos);
    }

    static bool is_prefix_path(const std::string& prefix, const std::string& path) {
        if (prefix == "/") {
            return true;
        }
        if (path.size() < prefix.size()) {
            return false;
        }
        if (path.compare(0, prefix.size(), prefix) != 0) {
            return false;
        }
        return path.size() == prefix.size() || path[prefix.size()] == '/';
    }

    static std::string hash_path_key(const std::vector<uint64_t>& hashes) {
        std::string key;
        key.reserve(hashes.size() * sizeof(uint64_t));
        for (uint64_t h : hashes) {
            key.append(reinterpret_cast<const char*>(&h), sizeof(h));
        }
        return key;
    }

    static std::string rewrite_destination_path(const std::string& source_path,
                                                const std::string& source_root,
                                                const std::string& destination_root) {
        if (source_path == source_root) {
            return destination_root;
        }
        if (!is_prefix_path(source_root, source_path)) {
            return std::string();
        }
        return destination_root + source_path.substr(source_root.size());
    }

    static std::string reconstruct_path_under_root(
        const std::string& root_path,
        const std::vector<uint64_t>& root_hashes,
        const std::vector<uint64_t>& full_hashes,
        const std::map<std::string, namespace_entry>& source_path_map) {
        if (full_hashes.size() < root_hashes.size()) {
            return std::string();
        }
        for (size_t i = 0; i < root_hashes.size(); ++i) {
            if (full_hashes[i] != root_hashes[i]) {
                return std::string();
            }
        }

        std::string path = root_path;
        for (size_t i = root_hashes.size(); i < full_hashes.size(); ++i) {
            std::vector<uint64_t> prefix_hashes(full_hashes.begin(), full_hashes.begin() + i + 1);
            auto it = source_path_map.find(hash_path_key(prefix_hashes));
            if (it == source_path_map.end()) {
                return std::string();
            }
            path.push_back('/');
            path += entry_name(it->second);
        }
        return path;
    }

    static constexpr const char* kPersistenceDir = "/mnt/batchtest/lhm";
    static constexpr const char* kPersistenceFile = "/mnt/batchtest/lhm/lhm_namespace.meta";
    static constexpr uint8_t kInodeFlagDeleted = 0x1;

    static uint64_t now_ns() {
        using namespace std::chrono;
        return static_cast<uint64_t>(
            duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
    }

    static uint64_t packed_inode_id(const inode_ref& ref) {
        return (static_cast<uint64_t>(ref.block_id) << 32) | static_cast<uint64_t>(ref.offset);
    }

    static bool is_zero_inode_ref(const inode_ref& ref) {
        return ref.block_id == 0 && ref.offset == 0;
    }

    bool allocate_inode_ref_for_create(const inode_ref& requested_ref, inode_ref& out_ref) {
        if (persistence_available_) {
            return persistent_store_.AllocateInodeSlot(out_ref);
        }

        if (!is_zero_inode_ref(requested_ref)) {
            out_ref = requested_ref;
            return true;
        }

        uint64_t ticket = transient_inode_cursor_.fetch_add(1, std::memory_order_relaxed);
        if (ticket == 0) {
            ticket = transient_inode_cursor_.fetch_add(1, std::memory_order_relaxed);
        }
        out_ref.block_id = static_cast<uint32_t>(ticket >> 32);
        out_ref.offset = static_cast<uint32_t>(ticket & 0xffffffffULL);
        return true;
    }

    void initialize_persistence_store() {
        persistence_available_ = false;
        transient_inode_cursor_.store(1, std::memory_order_relaxed);
        root_persistent_ref_ = make_inode_ref(0, 0);
        root_persistent_ref_valid_ = false;

        if (const char* env = std::getenv("LHM_ENABLE_L0_CACHE")) {
            if (env[0] == '0') {
                l0_cache_enabled_ = false;
            } else if (env[0] == '1') {
                l0_cache_enabled_ = true;
            }
        }

        if (::mkdir(kPersistenceDir, 0755) != 0 && errno != EEXIST) {
            return;
        }
        if (!persistent_store_.OpenOrCreate(kPersistenceFile)) {
            return;
        }
        if (!persistent_store_.set_l0_cache_enabled(l0_cache_enabled_)) {
            persistent_store_.Close();
            return;
        }
        persistence_available_ = ensure_root_inode_persisted();
    }

    bool ensure_root_inode_persisted() {
        if (root_persistent_ref_valid_) {
            return true;
        }

        inode_ref root_slot;
        if (!persistent_store_.AllocateInodeSlot(root_slot)) {
            return false;
        }

        inode_disk root{};
        root.inode_id = packed_inode_id(root_slot);
        root.parent_inode_id = 0;
        inode_ref root_dir_head = make_inode_ref(0, 0);
        if (!persistent_store_.CreateDirectoryChain(root.inode_id, root_dir_head)) {
            return false;
        }
        root.primary_ref = root_dir_head;
        root.aux_ref = root_dir_head;
        root.ctime_ns = now_ns();
        root.mtime_ns = root.ctime_ns;
        root.tombstone_epoch = 0;
        root.stable_version = 1;
        root.generation = 1;
        root.link_count = 1;
        root.mode = 0755;
        root.type = static_cast<uint8_t>(inode_type::directory);
        root.flags = 0;
        root.checksum = 0;
        if (!persistent_store_.WriteInode(root_slot, root)) {
            return false;
        }

        root_persistent_ref_ = root_slot;
        root_persistent_ref_valid_ = true;
        return true;
    }

    bool resolve_directory_persistent_ref(const std::string& normalized_path,
                                          inode_ref& out,
                                          threadinfo& ti) const {
        if (normalized_path == "/") {
            if (!root_persistent_ref_valid_) {
                return false;
            }
            out = root_persistent_ref_;
            return true;
        }
        value_type dir_entry;
        if (!lookup_directory(normalized_path, dir_entry, ti)) {
            return false;
        }
        out = dir_entry.ref;
        return true;
    }

    bool resolve_parent_persistent_ref(const ParsedPath& parsed,
                                       inode_ref& out,
                                       threadinfo& ti) const {
        return resolve_directory_persistent_ref(parent_path(parsed.normalized_path), out, ti);
    }

    bool resolve_parent_creation_context(const ParsedPath& parsed,
                                         node_type*& parent_root,
                                         inode_ref& parent_ref,
                                         threadinfo& ti) const {
        parent_root = nullptr;
        parent_ref = make_inode_ref(0, 0);
        if (parsed.normalized_path == "/" || parsed.components.empty()) {
            return false;
        }

        ParsedPath parent = parsed;
        parent.normalized_path = parent_path(parsed.normalized_path);
        parent.components.pop_back();
        parent.hashes.pop_back();

        const node_type* parent_root_const = nullptr;
        if (!locate_directory_root(parent, parent_root_const, ti)) {
            return false;
        }
        parent_root = const_cast<node_type*>(parent_root_const);

        if (parent.normalized_path == "/") {
            if (persistence_available_) {
                if (!root_persistent_ref_valid_) {
                    return false;
                }
                parent_ref = root_persistent_ref_;
            }
            return true;
        }

        if (!parent_root->has_directory_meta()) {
            return false;
        }
        const directory_meta* meta = parent_root->directory_meta();
        if (meta == nullptr || !directory_meta_name_equals(*meta, parent.components.back())) {
            return false;
        }
        parent_ref = meta->ref;
        return true;
    }

    bool create_file_on_parent_root(const ParsedPath& parsed, node_type* parent_root, inode_ref ref,
                                    threadinfo& ti) {
        value_type entry = make_namespace_entry(entry_kind::file, ref, parsed.components.back());
        single_component_key key = make_single_component_key(parsed.hashes.back());

        if (fast_mode_) {
            cursor_type cursor(parent_root, key.as_str().data(), key.as_str().length());
            bool already_present = cursor.find_insert(ti);
            if (already_present) {
                if (cursor.is_layer()) {
                    cursor.finish_read();
                    return false;
                }
                cursor.value() = entry;
                fence();
                cursor.finish(0, ti);
                return true;
            }
            cursor.value() = entry;
            fence();
            cursor.finish(1, ti);
            return true;
        }

        cursor_type cursor(parent_root, key.as_str().data(), key.as_str().length());
        bool already_present = cursor.find_insert(ti);
        if (already_present) {
            if (cursor.is_layer()) {
                cursor.finish_read();
                return false;
            }
            cursor.finish(0, ti);
            return false;
        }
        cursor.value() = entry;
        fence();
        cursor.finish(1, ti);
        return true;
    }

    bool create_directory_on_parent_root(const ParsedPath& parsed, node_type* parent_root,
                                         inode_ref ref, threadinfo& ti) {
        single_component_key edge_key = make_single_component_key(parsed.hashes.back());
        cursor_type cursor(parent_root, edge_key.as_str().data(), edge_key.as_str().length());
        node_type* directory_root = nullptr;
        directory_meta meta = make_directory_meta(ref, parsed.hashes.back(),
                                                  parsed.components.back());

        if (!cursor.create_layer_with_meta(directory_root, meta, ti)) {
            return false;
        }
        return true;
    }

    bool persist_create_success(const ParsedPath& parsed, entry_kind kind, inode_ref persistent_ref,
                                const inode_ref& parent_ref) {
        if (!persistence_available_) {
            return true;
        }
        if (parsed.normalized_path == "/") {
            return true;
        }

        inode_disk inode{};
        const uint64_t ts = now_ns();
        inode.inode_id = packed_inode_id(persistent_ref);
        inode.parent_inode_id = packed_inode_id(parent_ref);
        inode.primary_ref = persistent_ref;
        inode.aux_ref = make_inode_ref(0, 0);
        if (kind == entry_kind::directory) {
            inode_ref dir_head = make_inode_ref(0, 0);
            if (!persistent_store_.CreateDirectoryChain(inode.inode_id, dir_head)) {
                return false;
            }
            inode.primary_ref = dir_head;
            inode.aux_ref = dir_head;
        }
        inode.ctime_ns = ts;
        inode.mtime_ns = ts;
        inode.tombstone_epoch = 0;
        inode.stable_version = 1;
        inode.generation = 1;
        inode.link_count = 1;
        inode.mode = (kind == entry_kind::directory) ? 0755 : 0644;
        inode.type = static_cast<uint8_t>(
            kind == entry_kind::directory ? inode_type::directory : inode_type::file);
        inode.flags = 0;
        inode.checksum = 0;
        return persistent_store_.WriteInode(persistent_ref, inode);
    }

    bool persist_delete_success(const value_type& existing, threadinfo&) {
        if (!persistence_available_) {
            return true;
        }

        inode_disk inode{};
        if (!persistent_store_.ReadInode(existing.ref, inode)) {
            return false;
        }
        const uint64_t ts = now_ns();
        inode.mtime_ns = ts;
        inode.tombstone_epoch = ts;
        inode.primary_ref = existing.ref;
        inode.link_count = 0;
        inode.stable_version = inode.stable_version + 1;
        inode.generation = inode.generation + 1;
        inode.flags = static_cast<uint8_t>(inode.flags | kInodeFlagDeleted);
        inode.checksum = 0;
        if (!persistent_store_.WriteInode(existing.ref, inode)) {
            return false;
        }
        return true;
    }

    bool persist_rename_success(const ParsedPath& old_parsed, const ParsedPath& new_parsed,
                                const value_type& moved_entry, threadinfo& ti) {
        (void) old_parsed;
        if (!persistence_available_) {
            return true;
        }

        inode_ref parent_ref = make_inode_ref(0, 0);
        if (!resolve_parent_persistent_ref(new_parsed, parent_ref, ti)) {
            return false;
        }

        inode_disk inode{};
        if (!persistent_store_.ReadInode(moved_entry.ref, inode)) {
            return false;
        }
        inode.parent_inode_id = packed_inode_id(parent_ref);
        inode.mtime_ns = now_ns();
        inode.stable_version = inode.stable_version + 1;
        inode.generation = inode.generation + 1;
        inode.checksum = 0;
        return persistent_store_.WriteInode(moved_entry.ref, inode);
    }

    bool create_entry(const std::string& path, entry_kind kind, inode_ref ref, threadinfo& ti) {
        ParsedPath parsed = PathKey::parse_absolute_path(path);
        return create_entry_from_parsed(parsed, kind, ref, ti);
    }

    bool force_remove_child_slot_fast(node_type* parent_root, uint64_t child_hash,
                                      threadinfo& ti) {
        single_component_key key = make_single_component_key(child_hash);
        cursor_type cursor(parent_root, key.as_str().data(), key.as_str().length());
        cursor.find_locked_edge(ti);
        int state = cursor.state();
        if (state < 0 && cursor.is_layer()) {
            return cursor.remove_layer_edge(ti);
        }
        if (state > 0) {
            cursor.finish(-1, ti);
            return true;
        }
        cursor.finish_read();
        return true;
    }

    bool remove_entry_fast_from_parsed(const ParsedPath& parsed, value_type* removed_out,
                                       threadinfo& ti) {
        ParsedPath parent = parsed;
        parent.normalized_path = parent_path(parsed.normalized_path);
        parent.components.pop_back();
        parent.hashes.pop_back();

        const node_type* parent_root_const = nullptr;
        if (!locate_directory_root(parent, parent_root_const, ti)) {
            return false;
        }
        node_type* parent_root = const_cast<node_type*>(parent_root_const);

        single_component_key key = make_single_component_key(parsed.hashes.back());
        cursor_type cursor(parent_root, key.as_str().data(), key.as_str().length());
        cursor.find_locked_edge(ti);
        int state = cursor.state();

        if (state < 0 && cursor.is_layer()) {
            if (removed_out != nullptr) {
                node_type* child_root = canonicalize_directory_root(cursor.layer_root());
                if (!child_root->has_directory_meta()) {
                    cursor.finish_read();
                    return false;
                }
                const directory_meta* meta = child_root->directory_meta();
                if (meta == nullptr) {
                    cursor.finish_read();
                    return false;
                }
                *removed_out = make_namespace_entry(entry_kind::directory, meta->ref,
                                                    directory_meta_name(*meta));
            }
            return cursor.remove_layer_edge(ti);
        }
        if (state <= 0) {
            cursor.finish_read();
            return false;
        }

        value_type slot_value = cursor.value();
        if (removed_out != nullptr) {
            *removed_out = slot_value;
        }
        cursor.finish(-1, ti);
        return true;
    }

    bool create_entry_fast_from_parsed(const ParsedPath& parsed, entry_kind kind, inode_ref ref,
                                       threadinfo& ti) {
        if (kind == entry_kind::directory) {
            return create_directory_fast_from_parsed(parsed, ref, ti);
        }
        if (parsed.normalized_path == "/" || parsed.components.empty()) {
            return false;
        }
        if (!entry_name_fits(parsed.components.back())) {
            return false;
        }

        ParsedPath parent = parsed;
        parent.normalized_path = parent_path(parsed.normalized_path);
        parent.components.pop_back();
        parent.hashes.pop_back();

        const node_type* parent_root_const = nullptr;
        if (!locate_directory_root(parent, parent_root_const, ti)) {
            return false;
        }
        node_type* parent_root = const_cast<node_type*>(parent_root_const);
        value_type entry = make_namespace_entry(kind, ref, parsed.components.back());
        single_component_key key = make_single_component_key(parsed.hashes.back());

        cursor_type cursor(parent_root, key.as_str().data(), key.as_str().length());
        bool already_present = cursor.find_insert(ti);
        if (already_present) {
            if (cursor.is_layer()) {
                cursor.finish_read();
                return false;
            }
            cursor.value() = entry;
            fence();
            cursor.finish(0, ti);
            return true;
        }

        cursor.value() = entry;
        fence();
        cursor.finish(1, ti);
        return true;
    }

    bool create_directory_fast_from_parsed(const ParsedPath& parsed, inode_ref ref,
                                           threadinfo& ti) {
        if (parsed.normalized_path == "/" || parsed.components.empty()) {
            return false;
        }
        if (!entry_name_fits(parsed.components.back())) {
            return false;
        }

        ParsedPath parent = parsed;
        parent.normalized_path = parent_path(parsed.normalized_path);
        parent.components.pop_back();
        parent.hashes.pop_back();

        const node_type* parent_root_const = nullptr;
        if (!locate_directory_root(parent, parent_root_const, ti)) {
            return false;
        }
        node_type* parent_root = const_cast<node_type*>(parent_root_const);

        single_component_key edge_key = make_single_component_key(parsed.hashes.back());
        cursor_type cursor(parent_root, edge_key.as_str().data(), edge_key.as_str().length());
        node_type* directory_root = nullptr;
        directory_meta meta = make_directory_meta(ref, parsed.hashes.back(),
                                                  parsed.components.back());
        if (!cursor.create_layer_with_meta(directory_root, meta, ti)) {
            return false;
        }
        return true;
    }

    bool rename_path_fast_from_parsed(const ParsedPath& old_parsed,
                                      const ParsedPath& new_parsed,
                                      threadinfo& ti,
                                      value_type* moved_entry_out) {
        if (old_parsed.normalized_path == "/" || new_parsed.normalized_path == "/") {
            return false;
        }
        if (old_parsed.normalized_path == new_parsed.normalized_path) {
            return true;
        }
        if (is_prefix_path(old_parsed.normalized_path, new_parsed.normalized_path)) {
            return false;
        }

        value_type source_entry;
        if (!lookup_entry_from_parsed(old_parsed, source_entry, ti)) {
            return false;
        }
        if (moved_entry_out != nullptr) {
            *moved_entry_out = source_entry;
        }
        if (entry_is_directory(source_entry)) {
            return rename_directory_o1_fast_from_parsed(old_parsed, new_parsed, ti);
        }

        if (!create_entry_from_parsed(new_parsed, source_entry.kind, source_entry.ref, ti)) {
            return false;
        }
        return remove_entry(old_parsed.normalized_path, ti);
    }

    bool rename_directory_o1_fast_from_parsed(const ParsedPath& old_parsed,
                                              const ParsedPath& new_parsed,
                                              threadinfo& ti) {
        if (old_parsed.normalized_path == "/" || new_parsed.normalized_path == "/") {
            return false;
        }
        if (old_parsed.normalized_path == new_parsed.normalized_path) {
            return true;
        }
        if (is_prefix_path(old_parsed.normalized_path, new_parsed.normalized_path)) {
            return false;
        }
        if (new_parsed.components.empty() || !entry_name_fits(new_parsed.components.back())) {
            return false;
        }

        ParsedPath old_parent = old_parsed;
        old_parent.normalized_path = parent_path(old_parsed.normalized_path);
        old_parent.components.pop_back();
        old_parent.hashes.pop_back();

        ParsedPath new_parent = new_parsed;
        new_parent.normalized_path = parent_path(new_parsed.normalized_path);
        new_parent.components.pop_back();
        new_parent.hashes.pop_back();

        const node_type* old_parent_root_const = nullptr;
        if (!locate_directory_root(old_parent, old_parent_root_const, ti)) {
            return false;
        }
        const node_type* new_parent_root_const = nullptr;
        if (!locate_directory_root(new_parent, new_parent_root_const, ti)) {
            return false;
        }

        directory_edge_lookup source_edge;
        if (!lookup_directory_edge_from_parent_root(const_cast<node_type*>(old_parent_root_const),
                                                   old_parsed.components.back(),
                                                   old_parsed.hashes.back(),
                                                   source_edge, ti)) {
            return false;
        }

        bool same_slot = old_parent_root_const == new_parent_root_const
            && old_parsed.hashes.back() == new_parsed.hashes.back();
        if (!same_slot) {
            if (!force_remove_child_slot_fast(const_cast<node_type*>(new_parent_root_const),
                                              new_parsed.hashes.back(), ti)) {
                return false;
            }
        }

        single_component_key target_edge_key = make_single_component_key(new_parsed.hashes.back());
        cursor_type attach_cursor(const_cast<node_type*>(new_parent_root_const),
                                  target_edge_key.as_str().data(),
                                  target_edge_key.as_str().length());
        if (!attach_cursor.attach_existing_layer(source_edge.child_root, ti)) {
            return false;
        }

        if (!source_edge.child_root->has_directory_meta()) {
            return false;
        }
        directory_meta* meta = source_edge.child_root->directory_meta();
        if (meta == nullptr) {
            return false;
        }
        *meta = make_directory_meta(meta->ref, new_parsed.hashes.back(),
                                    new_parsed.components.back());

        single_component_key old_edge_key = make_single_component_key(old_parsed.hashes.back());
        cursor_type detach_cursor(const_cast<node_type*>(old_parent_root_const),
                                  old_edge_key.as_str().data(),
                                  old_edge_key.as_str().length());
        detach_cursor.find_locked_edge(ti);
        int state = detach_cursor.state();
        if (state >= 0 || !detach_cursor.is_layer()
            || detach_cursor.layer_root() != source_edge.child_root) {
            detach_cursor.finish_read();
            return false;
        }
        if (!detach_cursor.remove_layer_edge(ti)) {
            return false;
        }
        return true;
    }

    bool rename_directory_o1_from_parsed(const ParsedPath& old_parsed,
                                         const ParsedPath& new_parsed,
                                         threadinfo& ti) {
        if (old_parsed.normalized_path == "/" || new_parsed.normalized_path == "/") {
            return false;
        }
        if (old_parsed.normalized_path == new_parsed.normalized_path) {
            return true;
        }
        if (is_prefix_path(old_parsed.normalized_path, new_parsed.normalized_path)) {
            return false;
        }
        if (new_parsed.components.empty() || !entry_name_fits(new_parsed.components.back())) {
            return false;
        }

        ParsedPath old_parent = old_parsed;
        old_parent.normalized_path = parent_path(old_parsed.normalized_path);
        old_parent.components.pop_back();
        old_parent.hashes.pop_back();

        ParsedPath new_parent = new_parsed;
        new_parent.normalized_path = parent_path(new_parsed.normalized_path);
        new_parent.components.pop_back();
        new_parent.hashes.pop_back();

        const node_type* old_parent_root_const = nullptr;
        if (!locate_directory_root(old_parent, old_parent_root_const, ti)) {
            return false;
        }
        const node_type* new_parent_root_const = nullptr;
        if (!locate_directory_root(new_parent, new_parent_root_const, ti)) {
            return false;
        }

        directory_edge_lookup source_edge;
        if (!lookup_directory_edge_from_parent_root(const_cast<node_type*>(old_parent_root_const),
                                                   old_parsed.components.back(),
                                                   old_parsed.hashes.back(),
                                                   source_edge, ti)) {
            return false;
        }

        value_type existing_target;
        if (lookup_child_from_parent_root(const_cast<node_type*>(new_parent_root_const),
                                          new_parsed.components.back(),
                                          new_parsed.hashes.back(),
                                          existing_target, ti)) {
            return false;
        }

        PathKey target_edge_key({new_parsed.hashes.back()});
        cursor_type attach_cursor(const_cast<node_type*>(new_parent_root_const),
                                  target_edge_key.as_str().data(),
                                  target_edge_key.as_str().length());
        if (!attach_cursor.attach_existing_layer(source_edge.child_root, ti)) {
            return false;
        }

        if (!source_edge.child_root->has_directory_meta()) {
            return false;
        }
        directory_meta* meta = source_edge.child_root->directory_meta();
        if (meta == nullptr) {
            return false;
        }
        *meta = make_directory_meta(meta->ref, new_parsed.hashes.back(),
                                    new_parsed.components.back());

        ParsedPath detach_parent = old_parent;
        PathKey old_edge_key({old_parsed.hashes.back()});
        cursor_type detach_cursor(const_cast<node_type*>(old_parent_root_const),
                                  old_edge_key.as_str().data(),
                                  old_edge_key.as_str().length());
        detach_cursor.find_locked_edge(ti);
        int state = detach_cursor.state();
        if (state >= 0 || !detach_cursor.is_layer()
            || detach_cursor.layer_root() != source_edge.child_root) {
            detach_cursor.finish_read();
            return false;
        }
        if (!detach_cursor.remove_layer_edge(ti)) {
            return false;
        }
        return true;
    }

    bool remove_entry(const std::string& path, threadinfo& ti) {
        ParsedPath parsed = PathKey::parse_absolute_path(path);
        return remove_entry_from_parsed(parsed, nullptr, ti);
    }

    bool remove_entry_from_parsed(const ParsedPath& parsed, value_type* removed_out,
                                  threadinfo& ti) {
        if (parsed.normalized_path == "/" || parsed.components.empty()) {
            return false;
        }

        if (fast_mode_) {
            return remove_entry_fast_from_parsed(parsed, removed_out, ti);
        }

        ParsedPath parent = parsed;
        parent.normalized_path = parent_path(parsed.normalized_path);
        parent.components.pop_back();
        parent.hashes.pop_back();

        const node_type* parent_root_const = nullptr;
        if (!locate_directory_root(parent, parent_root_const, ti)) {
            return false;
        }
        node_type* parent_root = const_cast<node_type*>(parent_root_const);

        single_component_key key = make_single_component_key(parsed.hashes.back());
        cursor_type cursor(parent_root, key.as_str().data(), key.as_str().length());
        cursor.find_locked_edge(ti);
        int state = cursor.state();

        if (state < 0 && cursor.is_layer()) {
            node_type* child_root = canonicalize_directory_root(cursor.layer_root());
            if (!child_root->has_directory_meta()) {
                cursor.finish_read();
                return false;
            }
            const directory_meta* meta = child_root->directory_meta();
            if (meta == nullptr || !directory_meta_name_equals(*meta, parsed.components.back())) {
                cursor.finish_read();
                return false;
            }
            if (directory_root_has_children(child_root)) {
                cursor.finish_read();
                return false;
            }
            if (removed_out != nullptr) {
                *removed_out = make_namespace_entry(entry_kind::directory, meta->ref,
                                                    directory_meta_name(*meta));
            }
            return cursor.remove_layer_edge(ti);
        }

        if (state <= 0) {
            cursor.finish_read();
            return false;
        }

        value_type slot_value = cursor.value();
        if (!entry_name_equals(slot_value, parsed.components.back()) || !entry_is_file(slot_value)) {
            cursor.finish_read();
            return false;
        }
        if (removed_out != nullptr) {
            *removed_out = slot_value;
        }
        cursor.finish(-1, ti);
        return true;
    }

    bool lookup_entry_from_parsed(const ParsedPath& parsed, value_type& out, threadinfo& ti) const {
        if (parsed.normalized_path == "/") {
            out = make_namespace_entry(entry_kind::directory, make_inode_ref(0, 0), "/");
            return true;
        }
        if (parsed.components.empty() || parsed.hashes.empty()) {
            return false;
        }

        // stat/lookup 走单一路线：
        // 1) 先只定位到父目录 root；
        // 2) 再只查最后一级孩子槽位；
        // 3) 命中 layer 就按目录返回，命中 value 就按文件返回。
        // 这样避免了旧路径中“先整条路径按目录逐级判断，再走一次 value 查询”的双遍历。
        ParsedPath parent = parsed;
        parent.normalized_path = parent_path(parsed.normalized_path);
        parent.components.pop_back();
        parent.hashes.pop_back();

        const node_type* parent_root_const = nullptr;
        if (!locate_directory_root(parent, parent_root_const, ti)) {
            return false;
        }

        return lookup_child_from_parent_root(const_cast<node_type*>(parent_root_const),
                                             parsed.components.back(),
                                             parsed.hashes.back(),
                                             out, ti);
    }

    bool create_entry_from_parsed(const ParsedPath& parsed, entry_kind kind, inode_ref ref,
                                  threadinfo& ti) {
        if (fast_mode_) {
            return create_entry_fast_from_parsed(parsed, kind, ref, ti);
        }
        if (kind == entry_kind::directory) {
            return create_directory_from_parsed(parsed, ref, ti);
        }
        if (parsed.normalized_path == "/") {
            return false;
        }
        if (parsed.components.empty()) {
            return false;
        }
        if (!entry_name_fits(parsed.components.back())) {
            return false;
        }

        ParsedPath parent = parsed;
        parent.normalized_path = parent_path(parsed.normalized_path);
        parent.components.pop_back();
        parent.hashes.pop_back();

        const node_type* parent_root_const = nullptr;
        if (!locate_directory_root(parent, parent_root_const, ti)) {
            return false;
        }
        node_type* parent_root = const_cast<node_type*>(parent_root_const);

        value_type entry = make_namespace_entry(kind, ref, parsed.components.back());
        single_component_key key = make_single_component_key(parsed.hashes.back());

        cursor_type cursor(parent_root, key.as_str().data(), key.as_str().length());
        bool already_present = cursor.find_insert(ti);
        if (already_present) {
            if (cursor.is_layer()) {
                cursor.finish_read();
                return false;
            }
            cursor.finish(0, ti);
            return false;
        }

        cursor.value() = entry;
        fence();
        cursor.finish(1, ti);
        return true;
    }

    bool create_directory_from_parsed(const ParsedPath& parsed, inode_ref ref, threadinfo& ti) {
        if (fast_mode_) {
            return create_directory_fast_from_parsed(parsed, ref, ti);
        }
        if (parsed.normalized_path == "/") {
            return false;
        }
        if (parsed.components.empty()) {
            return false;
        }
        if (!entry_name_fits(parsed.components.back())) {
            return false;
        }

        ParsedPath parent = parsed;
        parent.normalized_path = parent_path(parsed.normalized_path);
        parent.components.pop_back();
        parent.hashes.pop_back();

        const node_type* parent_root_const = nullptr;
        if (!locate_directory_root(parent, parent_root_const, ti)) {
            return false;
        }
        node_type* parent_root = const_cast<node_type*>(parent_root_const);

        single_component_key edge_key = make_single_component_key(parsed.hashes.back());
        cursor_type cursor(parent_root, edge_key.as_str().data(), edge_key.as_str().length());
        node_type* directory_root = nullptr;
        directory_meta meta = make_directory_meta(ref, parsed.hashes.back(),
                                                  parsed.components.back());
        if (!cursor.create_layer_with_meta(directory_root, meta, ti)) {
            return false;
        }
        return true;
    }

    std::vector<subtree_record> scan_subtree(const ParsedPath& root, threadinfo& ti) const {
        subtree_scanner scanner;
        std::vector<subtree_record> results;
        scanner.root_hashes = &root.hashes;
        scanner.out = &results;
        table_.scan(lcdf::Str(""), false, scanner, ti);
        return results;
    }

    // 在父目录对应的 layer root 内，用“最后一级组件哈希 + 真实名字”检查孩子是否已存在。
    // 当前目录和文件都可以复用这条局部查找路径，避免每次都从全局根重新走完整路径。
    bool lookup_child_from_parent_root(node_type* parent_root, const std::string& child_name,
                                       uint64_t child_hash, value_type& out,
                                       threadinfo& ti) const {
        child_slot_lookup slot;
        if (!lookup_child_slot_from_parent_root(parent_root, child_hash, slot, ti)) {
            return false;
        }

        if (slot.is_layer) {
            const node_type* child_root = slot.child_root;
            if (child_root->has_directory_meta()) {
                const directory_meta* meta = child_root->directory_meta();
                if (meta != nullptr && directory_meta_name_equals(*meta, child_name)) {
                    out = make_namespace_entry(entry_kind::directory, meta->ref,
                                               directory_meta_name(*meta));
                    return true;
                }
            }
            return false;
        }

        const value_type& raw = slot.value;
        if (entry_name_equals(raw, child_name)) {
            out = raw;
            return true;
        }
        return false;
    }

    bool lookup_directory_edge_from_parent_root(node_type* parent_root,
                                                const std::string& child_name,
                                                uint64_t child_hash,
                                                directory_edge_lookup& out,
                                                threadinfo& ti) const {
        single_component_key child_key = make_single_component_key(child_hash);
        cursor_type cursor(parent_root, child_key.as_str().data(), child_key.as_str().length());
        cursor.find_locked_edge(ti);
        int state = cursor.state();
        if (state < 0 && cursor.is_layer()) {
            node_type* child_root = canonicalize_directory_root(cursor.layer_root());
            cursor.finish_read();
            if (!child_root->has_directory_meta()) {
                return false;
            }
            const directory_meta* meta = child_root->directory_meta();
            if (meta == nullptr || !directory_meta_name_equals(*meta, child_name)) {
                return false;
            }
            out.found = true;
            out.child_root = child_root;
            out.entry = make_namespace_entry(entry_kind::directory, meta->ref,
                                             directory_meta_name(*meta));
            return true;
        }
        cursor.finish_read();
        return false;
    }

    bool lookup_child_slot_from_parent_root(node_type* parent_root, uint64_t child_hash,
                                            child_slot_lookup& out, threadinfo& ti) const {
        out = child_slot_lookup{};
        single_component_key child_key = make_single_component_key(child_hash);
        unlocked_cursor_type cursor(parent_root, child_key.as_str().data(),
                                    child_key.as_str().length());
        cursor.find_unlocked_edge(ti);
        int state = cursor.state();
        if (state < 0 && cursor.is_layer()) {
            out.found = true;
            out.is_layer = true;
            out.child_root = canonicalize_directory_root(cursor.layer_root());
            return true;
        }
        if (state <= 0) {
            return false;
        }
        out.found = true;
        out.value = cursor.value();
        return true;
    }

    bool locate_directory_root(const ParsedPath& parsed, const node_type*& out,
                               threadinfo& ti) const {
        out = table_.root();
        if (parsed.normalized_path == "/") {
            return true;
        }

        for (uint64_t component_hash : parsed.hashes) {
            single_component_key edge_key = make_single_component_key(component_hash);
            unlocked_cursor_type cursor(const_cast<node_type*>(out), edge_key.as_str().data(),
                                        edge_key.as_str().length());
            cursor.find_unlocked_edge(ti);
            int state = cursor.state();
            if (state >= 0 || !cursor.is_layer()) {
                return false;
            }
            out = canonicalize_directory_root(cursor.layer_root());
        }
        return true;
    }

    void append_directory_children(const node_type* parent_root,
                                   const std::vector<uint64_t>& parent_hashes,
                                   std::vector<readdir_record>& out) const {
        append_directory_children_from_node(parent_root, parent_hashes, out);
    }

    void append_directory_children_from_node(const node_type* node,
                                             const std::vector<uint64_t>& parent_hashes,
                                             std::vector<readdir_record>& out) const {
        if (node->isleaf()) {
            const typename node_type::leaf_type* leaf =
                static_cast<const typename node_type::leaf_type*>(node);
            typename node_type::leaf_type::permuter_type perm = leaf->permutation();
            for (int i = 0; i != leaf->size(); ++i) {
                int p = perm[i];
                uint64_t child_hash = static_cast<uint64_t>(leaf->ikey(p));

                if (!leaf->is_layer(p)) {
                    value_type slot_value = leaf->lv_[p].value();
                    readdir_record record;
                    record.full_hashes = parent_hashes;
                    record.full_hashes.push_back(child_hash);
                    record.child_component_hash = child_hash;
                    record.child_name = entry_name(slot_value);
                    record.entry = slot_value;
                    out.push_back(std::move(record));
                    continue;
                }

                // 命中目录 edge 时只恢复当前目录项，不下钻子目录。
                const node_type* child_root =
                    canonicalize_directory_root(leaf->lv_[p].layer());
                if (!child_root->has_directory_meta()) {
                    continue;
                }
                const directory_meta* meta = child_root->directory_meta();
                if (meta == nullptr) {
                    continue;
                }
                readdir_record record;
                record.full_hashes = parent_hashes;
                record.full_hashes.push_back(meta->component_hash);
                record.child_component_hash = meta->component_hash;
                record.child_name = directory_meta_name(*meta);
                record.entry = make_namespace_entry(entry_kind::directory, meta->ref,
                                                    directory_meta_name(*meta));
                out.push_back(std::move(record));
            }
            return;
        }

        const typename node_type::internode_type* in =
            static_cast<const typename node_type::internode_type*>(node);
        for (int i = 0; i != in->size() + 1; ++i) {
            if (in->child_[i]) {
                append_directory_children_from_node(in->child_[i], parent_hashes, out);
            }
        }
    }
};

}  // namespace MasstreeLHM

namespace Masstree {

template <>
class value_print<MasstreeLHM::namespace_entry> {
  public:
    static void print(MasstreeLHM::namespace_entry value, FILE* f, const char* prefix,
                      int indent, Str key, kvtimestamp_t, char* suffix) {
        std::string name = MasstreeLHM::entry_name(value);
        fprintf(f, "%s%*s%.*s = {kind=%s, name=%s, ref=(block=%" PRIu32 ", offset=%" PRIu32 ")}%s\n",
                prefix, indent, "", key.len, key.s,
                MasstreeLHM::entry_kind_name(value.kind), name.c_str(),
                value.ref.block_id, value.ref.offset, suffix);
    }
};

}  // namespace Masstree

#endif
