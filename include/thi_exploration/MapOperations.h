
/* Copyright (c), Prof. Dr. Christian Pfitzner, All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
 */



#if !defined(MAP_OPERATIONS)
#define MAP_OPERATIONS


#include <nav_msgs/OccupancyGrid.h>
#include <thi_exploration/Frontier.h>


namespace thi
{
    constexpr auto FREE     =  0;
    constexpr auto UNKNOWN  = -1;  
    constexpr auto OCCUPIED =  100;

    constexpr auto UNVALID     =  0; 
    constexpr auto FRONTIER    =  100; 
    constexpr auto NO_FRONTIER =  50; 



namespace MapOperations
{

    static nav_msgs::OccupancyGrid copy(const nav_msgs::OccupancyGrid input)  
    {
        nav_msgs::OccupancyGrid output; 
        output.header.frame_id = input.header.frame_id; 
        output.header.seq      = input.header.seq; 
        output.header.stamp    = input.header.stamp; 
        output.info.height     = input.info.height; 
        output.info.width      = input.info.width;  
        output.info.height     = input.info.height; 
        output.info.resolution = input.info.resolution;
        
        return output; 
    }


    static unsigned int getIndexFromCoord(const nav_msgs::OccupancyGrid map, unsigned int col, unsigned int row) 
    {
        return map.info.width*row + col; 
    }


    static std::array<unsigned int, 2> getCoordFromIndex(const nav_msgs::OccupancyGrid map, unsigned int idx)
    {
        std::array<unsigned int, 2> coord; 

        coord[0] = idx / map.info.width; 
        coord[1] = idx % map.info.height; 

        return coord; 
    }


    static 
    unsigned int getNrCellsInMap(const nav_msgs::OccupancyGrid map, int state)
    {
        auto counter = 0; 
        
        // iterate over all frontier cells 
        const auto nr_of_cells = map.info.height*map.info.width; 

        for (auto  i = 0; i < nr_of_cells; i++)
        {
            const auto current_cell = map.data[i];
            if(current_cell == state)
            {
                counter++; 
            }
        }

        return counter; 
    }



    static 
    unsigned int getUnknownCellsInMap( const nav_msgs::OccupancyGrid map)
    {
        return getNrCellsInMap(map, UNKNOWN); 
    }

    static 
    unsigned int getFreeCellsInMap(const nav_msgs::OccupancyGrid map)
    {
        return getNrCellsInMap(map, FREE); 
    }

    static 
    unsigned int getOccupiedCellsInMap(const nav_msgs::OccupancyGrid map)
    {
        return getNrCellsInMap(map, OCCUPIED); 
    }

    static
    unsigned int getFrontierCellsInMap(const nav_msgs::OccupancyGrid map)
    {
        return getNrCellsInMap(map, FRONTIER); 
    }


    static 
    bool neighborIsUnknown(const std::vector<int> neighbors, const nav_msgs::OccupancyGrid map) 
    {
        for (auto i = 0; i < neighbors.size() ; i++)
        {
            if(map.data[neighbors[i]] == UNKNOWN)    return true; 
        }

        return false; 
    }



    static 
    bool idxInRange(const unsigned int idx, const nav_msgs::OccupancyGrid map)
    {
        const auto map_size = map.info.width*map.info.height; 

        // check if the index within the range of the map
        if(idx < 0)        return false;    
        if(idx > map_size) return false; 

        return true; 
    }


    static 
    std::vector<int> getNeighborIndices(const unsigned int idx, const nav_msgs::OccupancyGrid map, bool get8Neighbors /* =  false */) 
    {
        std::vector<int> neighbors; 

        const auto width  = map.info.width; 
        const auto height = map.info.height; 

        // top
        {
            const auto top_idx     = idx - width; 
            if(idxInRange(top_idx, map))            neighbors.push_back(top_idx); 
            else                                    neighbors.push_back(thi::UNVALID); 
        }

        // bottom
        {
            const auto bottom_idx  = idx + width; 
            if(idxInRange(bottom_idx, map))         neighbors.push_back(bottom_idx); 
            else                                    neighbors.push_back(thi::UNVALID); 
        }
        
        // left
        {
            const auto left_idx    = idx - 1; 
            if(idxInRange(left_idx, map))           neighbors.push_back(left_idx); 
            else                                    neighbors.push_back(thi::UNVALID); 
        }

        // right
        {
            const auto right_idx   = idx + 1; 
            if(idxInRange(right_idx, map))          neighbors.push_back(right_idx); 
            else                                   neighbors.push_back(thi::UNVALID);
        }

        if(get8Neighbors == true)
        {
            // top left
            {
                const auto top_left_idx     = idx - width - 1; 
                if(idxInRange(top_left_idx, map))           neighbors.push_back(top_left_idx); 
                else                                        neighbors.push_back(thi::UNVALID); 
            }

            // top right
            {
                const auto top_right_idx     = idx - width + 1; 
                if(idxInRange(top_right_idx, map))          neighbors.push_back(top_right_idx); 
                else                                        neighbors.push_back(thi::UNVALID); 
            }

            // bottom left
            {
                const auto bottom_left_idx     = idx + width - 1; 
                if(idxInRange(bottom_left_idx, map))        neighbors.push_back(bottom_left_idx); 
                else                                        neighbors.push_back(thi::UNVALID); 
            }

            // bottom right
            {
                const auto bottom_right_idx     = idx + width + 1; 
                if(idxInRange(bottom_right_idx, map))       neighbors.push_back(bottom_right_idx); 
                else                                        neighbors.push_back(thi::UNVALID); 
            }

        }
        return neighbors; 
    };


}; 


}; 


#endif /* MAP_OPERATIONS*/