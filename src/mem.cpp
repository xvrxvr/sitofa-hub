#include "common.h"
#include "mem.h"

#include <sys/mman.h>

/*
    static constexpr int MinBlockSize = 16; // Minimum block size
    static constexpr int MemPageSize = 4096; // Memory page size. Used to round all memory allocation requests

    // Type of index into shared memory. Expressed in MinBlockSize chanks
    using MemoryIndexType = uint32_t;

    // List of Free blocks (by size)
    // Index 0 corresponded by block of MinBlockSize bytes, 1 for MinBlockSize*2 bytes and so on
    // Contents - scaled index of block (shift in memory image is <scaled-index>*<block-size>)
    std::vector<std::unordered_set<MemoryIndexType>> free_lists;

    struct AllocatedBlockDsc {
        uint8_t block_level=0; // block size is 16 << block_level
        uint8_t options=0;
    };

    // All allocated blocks. Index - shift i memory image (in 16 bytes chanks)
    // Data - block description
    std::unordered_map<MemoryIndexType, AllocatedBlockDsc> allocated;

    // First not yet allocated block of maximum size (in maximum block size chanks)
    MemoryIndexType first_to_allocate;

    // Maximum memory size (VM area). In maximum block size chanks
    MemoryIndexType max_memory_size;

    // Current memory size (size of file). In maximum block size chanks
    MemoryIndexType cur_memory_size;

    // Maximum allocation size
    MemoryIndexType max_block_size;

    // Size of maximim block
    size_t max_block_size() const {return MinBlockSize << (free_lists.size()-1);}

    uint8_t* buf_memory; // Mapped shared memory file
    int buf_fd; // File descriptor of shared file
*/

#define ERROR(msg)  throw Utils::Exception("SharedMem: " msg)
#define ERRORF(msg, ...)  throw Utils::Exception(std::format(msg, __VA_ARG__))

inline size_t round_up(size_t who, size_t upto) {return (who + upto - 1) / upto * upto;}

// Adjust requested size (size) with minimum (min_size) and round to requestewd block size and Memory Page size 
size_t MemAlloc::adjust_size(size_t size, size_t min_size) {return round_up(round_up(std::max(size, min_size), max_block_size()), MemPageSize);}

// max_block_size - maximum size of memory that can be requested
MemAlloc::MemAlloc(size_t max_block_size, size_t allocate_mem, size_t reserve_mem, int client_pid) : free_lists(size2index(max_block_size)+1)
{
    cur_memory_size = adjust_size(allocate_mem, max_block_size()*2);
    max_memory_size = adjust_size(reserve_mem, cur_memory_size);

    buf_fd = memfd_create(std::format("shared-mem-{}", client_pid).c_str(), MFD_CLOEXEC);
    if (buf_fd == -1) ERRORF("Can't create shared temporary file for client {}", client_pid);
    if (-1 == ftruncate(buf_fd, cur_memory_size)) ERRORF("Can't set size for Shared Memory File to {} (client {})", cur_memory_size, client_fd);

    buf_memory = (uint8_t*)mmap(NULL, max_memory_size, PROT_READ|PROT_WRITE, MAP_SHARED, buf_fd, 0);
    if (!buf_memory) ERRORF("Can't map Shared Memory File to memory (with size {}) for client {}", max_memory_size, client_id);
}

MemAlloc::~MemAlloc()
{
    if (buf_memory) munmap(buf_memory, max_memory_size);
    if (buf_fd != -1) close(buf_fd);

}

// Find level of required memory. 'size' <= MinBlockSize << returned-value
// If required level is more than maximum allocated - emit error and raise exception
int MemAlloc::size2index(size_t size)
{
    if (size <= MinBlockSize) return 0;
    unsigned sz = (size + MinBlockSize - 1) / MinBlockSize;
    int lead_zero = __builtin_clz(sz);
    int bit_pos = 8*sizeof(sz) - 1 - lead_zero;
    if ((1<<bit_pos) >= sz) ++bit_pos;
    if (!free_lists.empty() && free_lists.size() <= bit_pos)
    {
        ERRORF("alloc: Requested block size {} is more, than maximum allowed {}", size, max_block_size());
    }
    return bit_pos;
}

// Extract one empty element from appropriate level list
// Return it or NoEntry constant if no empty entry
MemAlloc::MemoryIndexType MemAlloc::take_free(int level)
{
    auto & cur_free = free_lists[level];
    if (cur_free.empty()) return NoEntry;
    auto pos = cur_free.begin();
    auto result = *pos;
    cur_free.erase(pos);
    return result;
}

// Split 'start_index' taken from level 'from_level' in pairs, down to 'to_level'
// Return last index (in to_level). All splitted parts put in list of free entries
MemAlloc::MemoryIndexType MemAlloc::split_down(int from_level, int to_level, MemoryIndexType index)
{
    assert(from_level > to_level);
    while(from_level != to_level)
    {
        index <<= 1;
        from_level--;
        free_on_level(index + 1, from_level);
    }
    return index;
}

// Put entry to free list
void MemAlloc::free_on_level(MemoryIndexType index, int level)
{
    auto & cur_free = free_lists[level];
    if (cur_free.count(index))
    {
        warning(std::format("Double free for block size {}: {}", MinBlockSize << level, index));
    }
    else if (level == cur_free.size() - 1 && index + 1 == first_to_allocate) --first_to_allocate;
    else cur_free.erase(level);
}


// Allocate memory block. Returns shift in memory image and pointer.
// Raise exception if can't allocate of requested size is more than maximum allwed (in later case error also printed)
// If 'can_fail' is true returns {-1, NULL} in case of memory overflow. Invalid memory request still raise exception
std::pair<size_t, void*> MemAlloc::alloc(size_t size, bool can_fail)
{
    size_t index_in_level; // Index in appropriate level for returned element
    int level = size2index(size);
    auto index_in_level = take_free(level);
    if (index_in_level == NoEntry) // No free entry - try to fine one on apper level and split it
    {
       for(int try_level = level+1; try_level < free_lists.size(); ++try_level)
       {
            index_in_level = take_free(try_level);
            if (index_in_level != NoEntry) // Take it and split down
            {
                index_in_level = split_down(try_level, level, index_in_level);
                break;
            }
       } 
    }
    if (index_in_level == NoEntry) // Try to allocate from anallocated yet space
    {
        if (first_to_allocate >= cur_memory_size) // No space, enlarge or fail
        {
            if (first_to_allocate >= max_memory_size) // No more space - fail
            {
                if (!can_fail) ERROR("No more memory");
                error("No more memory");
                return {-1u, NULL};
            }
            // Expand Shared Memory file
            auto new_size = std::min(max_memory_size * max_block_size(), round_up((first_to_allocate+1) * max_block_size(), MemPageSize));
            if (ftruncate(buf_fd, new_size) == -1) // Can't aloocate - emit warning and try again with minimum required size
            {
                warning(std::format("SharedMem:alloc: Can't extend Shared Memory file to {} bytes", new_size));
                auto nnew_size = std::min(max_memory_size * max_block_size(), round_up((first_to_allocate) * max_block_size(), MemPageSize));
                if (nnew_size == new_size || ftruncate(buf_fd, nnew_size) == -1) // Still can't allocate
                {
                    if (can_fail) return {-1u, NULL};
                    ERRORF("Shared memory file expanding fail (requested size is {})", nnew_size);
                }
                new_size = nnew_size;
            }
            cur_memory_size = new_size / max_block_size();
        }
        index_in_level = split_down(free_lists.size()-1, level, first_to_allocate++);
    }
    index_in_level <<= level; // Adjust index to MinBlockSize chunks
    assert(!allocated.count(index_in_level));
    allocated[index_in_level] = AllocatedBlockDsc{level};
    size_t byte_shift = index_in_level * MinBlockSize;
    return {byte_shift, buf_memory+byte_shift};
}

// Free block by shift in memory. Writes warning on trying to free alread freed block.
// Raise exception on trying to free not allocated block (and writes error)
void MemAlloc::free_by_shift(size_t shift)
{
    // Bad pointer check
    if (shift & (MinBlockSize-1)) ERRORF("free: Badly aligned pointer {:x}", shift);
    if (shift / max_block_size() >= cur_memory_size) ERRORF("free: Pointer value is out of memory block {:x}", shift);
    shift /= MinBlockSize;
    if (!allocated.count(shift)) ERRORF("free: Try to free unallocated block {:x}", shift*MinBlockSize);
    int level = allocated[shift].level;
    allocated.erase(shift);
    free_on_level(shift >>= level, level);
}
