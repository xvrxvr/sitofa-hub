#pragma once
#include "interf.h"

/*
Shared memory manager.

This module iomplements fast (power of 2) allocator algorithm.
Minimal memory size is 16 bytes, maximum defined by configuration.
Memory allocation granularity - 4K (page size)
*/

class MemAlloc : public Interf::ErrorReportable {
    static constexpr int MinBlockSize = 16; // Minimum block size
    static constexpr int MemPageSize = 4096; // Memory page size. Used to round all memory allocation requests

    // Type of index into shared memory. Expressed in MinBlockSize chanks
    using MemoryIndexType = uint32_t;
    static constexpr MemoryIndexType NoEntry = MemoryIndexType(-1); // Constant for absent entry

    // List of Free blocks (by size)
    // Index 0 corresponded by block of MinBlockSize bytes, 1 for MinBlockSize*2 bytes and so on
    // Contents - scaled index of block (shift in memory image is <scaled-index>*<block-size>)
    std::vector<std::unordered_set<MemoryIndexType>> free_lists;

    struct AllocatedBlockDsc {
        uint8_t block_level=0; // block size is MinBlockSize << block_level
        uint8_t options=0; // Not used for now
    };

    // All allocated blocks. Index - shift i memory image (in MinBlockSize bytes chanks)
    // Data - block description
    std::unordered_map<MemoryIndexType, AllocatedBlockDsc> allocated;

    // First not yet allocated block of maximum size (in maximum block size chanks)
    MemoryIndexType first_to_allocate=0;

    // Maximum memory size (VM area). In maximum block size chanks
    MemoryIndexType max_memory_size=0;

    // Current memory size (size of file). In maximum block size chanks
    MemoryIndexType cur_memory_size=0;

    // Size of maximim block
    size_t max_block_size() const {return MinBlockSize << (free_lists.size()-1);}

    uint8_t* buf_memory = NULL; // Mapped shared memory file
    int buf_fd = -1; // File descriptor of shared file

    // Find level of required memory. 'size' <= MinBlockSize << returned-value
    // If required level is more than maximum allocated - emit error and raise exception
    int size2level(size_t size);

    // Adjust requested size (size) with minimum (min_size) and round to requestewd block size and Memory Page size 
    size_t adjust_size(size_t size, size_t min_size);

    // Extract one empty element from appropriate level list
    // Return it or NoEntry constant if no empty entry
    MemoryIndexType take_free(int level);

    // Split 'start_index' taken from level 'from_level' in pairs, down to 'to_level'
    // Return last index (in to_level). All splitted parts put in list of free entries
    MemoryIndexType split_down(int from_level, int to_level, MemoryIndexType start_index);

    // Put entry to free list
    void free_on_level(MemoryIndexType, int level);

public:
    // max_block_size - maximum size of memory that can be requested
    // allocate_mem - initial zie of allocated memory
    // reserve_mem - maximum size of allocated memory
    // client_pid - PID of client (for debug purpose)
    MemAlloc(size_t max_block_size, size_t allocate_mem, size_t reserve_mem, int client_pid);
    ~MemAlloc();

    // Get shared memory fiule handle
    int get_fd() const {return buf_fd;}

    // Allocate memory block. Returns shift in memory image and pointer.
    // Raise exception if can't allocate of requested size is more than maximum allwed (in later case error also printed)
    // If 'can_fail' is true returns {-1, NULL} in case of memory overflow. Invalid memory request still raise exception
    std::pair<size_t, void*> alloc(size_t, bool can_fail=false);

    // Free block by shift in memory. Writes warning on trying to free alread freed block.
    // Raise exception on trying to free not allocated block (and writes error)
    void free_by_shift(size_t shift);
    void free_by_address(void* p) {if (p) free_by_shift(((uint8_t*)p)-buf_memory);}
};
