
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

#if !defined(EXPLORER)
#define EXPLORER


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

class Frontier
{
public: 
  unsigned int index_in_map; 
  unsigned int group; 
};





// const values for occupancy grid

namespace thi
{
  constexpr auto FREE     =  0;
  constexpr auto UNKNOWN  = -1;  
  constexpr auto OCCUPIED =  100;

  constexpr auto UNVALID     =  0; 
  constexpr auto FRONTIER    =  100; 
  constexpr auto NO_FRONTIER =  50; 


struct Point2D_uint
{
  unsigned int col; 
  unsigned int row; 
}; 


/**
 * @class     Explorer
 * @author    Prof. Dr. Christian Pfitzner
 * 
 */
class Explorer
{
  public:
  /**
   * @brief Construct a new Explorer object
   * @param nh      Node Handle of the ros node 
   */
    Explorer(ros::NodeHandle nh);

    /**
     * @brief Destroy the Explorer object
     */
    ~Explorer(void);

    /**
     * @brief Set the Map object
     * @param map 
     */
    void setMap(const nav_msgs::OccupancyGrid::ConstPtr map)
    {
      _map = *map; 
    }


    /**
     * @brief function to process exploration
     */
    void process(void)
    {
      // // resize map to speed up exploration
      // const nav_msgs::OccupancyGrid resizedMap      = this->resizeMap(_map, 0.1);


      ROS_ERROR_STREAM("Free cells:     " << this->getFreeCellsInMap(_map)    ); 
      ROS_ERROR_STREAM("occ  cells:     " << this->getOccupiedCellsInMap(_map)); 
      ROS_ERROR_STREAM("unkn cells:     " << this->getUnknownCellsInMap(_map) ); 

      // find frontiers
      const nav_msgs::OccupancyGrid frontier_map    = this->findFrontiers(_map);
      
      ROS_ERROR_STREAM("frontier cells: " << this->getFrontierCellsInMap(_map) ); 

      // group frontiers 
      std::vector< std::vector<Frontier> > groups = this->groupFrontiers(frontier_map);

      // // weight frontiers      
      // const std::vector<Frontier> frontier_weighted = this->weightFrontiers(frontier_grouped);
    }


private: 
    unsigned int getNrCellsInMap(const nav_msgs::OccupancyGrid map, int state); 


    unsigned int getUnknownCellsInMap( const nav_msgs::OccupancyGrid map)
    {
      return this->getNrCellsInMap(map, UNKNOWN); 
    }

    unsigned int getFreeCellsInMap(const nav_msgs::OccupancyGrid map)
    {
      return this->getNrCellsInMap(map, FREE); 
    }

    unsigned int getOccupiedCellsInMap(const nav_msgs::OccupancyGrid map)
    {
      return this->getNrCellsInMap(map, OCCUPIED); 
    }

    unsigned int getFrontierCellsInMap(const nav_msgs::OccupancyGrid map)
    {
      return this->getNrCellsInMap(map, FRONTIER); 
    }

    thi::Point2D_uint getCoordFromIndex(const nav_msgs::OccupancyGrid map, unsigned int idx) const
    {
      Point2D_uint coord; 

      coord.col = idx % map.info.height; 
      coord.row = idx / map.info.width; 

      return coord; 
    }


    unsigned int getIndexFromCoord(const nav_msgs::OccupancyGrid map, unsigned int col, unsigned int row) const 
    {
      unsigned int idx = 0; 

      idx = map.info.width*row + col; 

      return idx; 
    }


    /**
     * @brief Function to resize the occupancy grid
     * 
     * @param input_map           input map 
     * @param output_size         size of the new grid in meter
     * @return nav_msgs::OccupancyGrid 
     */
    nav_msgs::OccupancyGrid resizeMap(const nav_msgs::OccupancyGrid input_map, const double output_size) const
    {

      // generate output occupancy grid
      nav_msgs::OccupancyGrid output; 






      if(_resize_map_pub.getNumSubscribers() > 0)
      {
        _resize_map_pub.publish(output); 
      }

      return output; 
    }


    /**
     * @brief Function to check wheater a cell is valid or not
     * 
     * @param idx 
     * @param map 
     * @return true 
     * @return false 
     */
    bool idxInRange(const unsigned int idx, const nav_msgs::OccupancyGrid map) const; 

    /**
     * @brief Get the Neighbor Indices object
     * 
     * @param idx 
     * @param map 
     * @return std::vector<int> 
     */
    std::vector<int> getNeighborIndices(const unsigned int idx, const nav_msgs::OccupancyGrid map, bool get8Neighbors = false) const; 
    /**
     * @brief 
     * 
     * @param neighbors 
     * @param map 
     * @return 
     */
    bool neighborIsUnknown(const std::vector<int> neighbors, const nav_msgs::OccupancyGrid map) const; 

    /**
     * Function to copy a occupancy grid 
     */
    nav_msgs::OccupancyGrid copy(const nav_msgs::OccupancyGrid input) const; 

    /**
     * @brief 
     * 
     * @param map 
     * @return nav_msgs::OccupancyGrid 
     */
    nav_msgs::OccupancyGrid findFrontiers(const nav_msgs::OccupancyGrid map) const; 


    void floodFill(nav_msgs::OccupancyGrid* map, unsigned int idx, int oldValue, int newValue);


    /**
     * @brief 
     * 
     * @param map 
     * @return std::vector<Frontier> 
     */
    std::vector< std::vector<Frontier> > groupFrontiers(nav_msgs::OccupancyGrid map); 



    /**
     * @brief 
     * 
     * @param frontiers 
     * @return std::vector<Frontier> 
     */
    std::vector<Frontier> weightFrontiers(std::vector<Frontier> frontiers) const
    {
      std::vector<Frontier> weighted_frontiers; 





      return weighted_frontiers; 
    }

  private:
    ros::NodeHandle         _nh;      //!< node handle of ros node
    nav_msgs::OccupancyGrid _map;     //!< input of occupancy grid

    ros::Publisher          _resize_map_pub; 
    ros::Publisher          _frontier_map_pub; 
    ros::Publisher          _frontier_goal_pub; 
};


};


#endif // EXPLORER