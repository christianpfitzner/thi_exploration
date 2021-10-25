
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
#include <tf2_ros/transform_listener.h>


#include <thi_exploration/Frontier.h>



// const values for occupancy grid

namespace thi
{

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
    void setMap(const nav_msgs::OccupancyGrid::ConstPtr map) { _map = *map; }


    /**
     * @brief function to process exploration
     */
    void process(void); 


private: 

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
    std::vector< Frontier > groupFrontiers(nav_msgs::OccupancyGrid map); 



    /**
     * @brief 
     * 
     * @param frontiers 
     * @return std::vector<Frontier> 
     */
    std::vector<Frontier> weightFrontiers(std::vector<Frontier> frontiers) const; 


    void publishMarker(std::vector<Frontier> frontiers) const; 

    geometry_msgs::Point32 getRobotsPose(void);


  private:
    ros::NodeHandle         _nh;      //!< node handle of ros node
    nav_msgs::OccupancyGrid _map;     //!< input of occupancy grid

    ros::Publisher          _resize_map_pub; 
    ros::Publisher          _frontier_map_pub; 
    ros::Publisher          _frontier_goal_pub;


    // tf2_ros::Buffer            _tfBuffer;
    // tf2_ros::TransformListener _tfListener; 
};


};


#endif // EXPLORER