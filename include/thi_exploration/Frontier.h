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


#if !defined(__FRONTIER__)
#define __FRONTIER__


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>

#include <thi_exploration/MapOperations.h>



namespace thi
{

class Frontier
{
public: 
  unsigned int index_in_map; 
  unsigned int group; 


  unsigned int getSize(void) const
  {
    return _indices.size(); 
  }


  void setMap(const nav_msgs::OccupancyGrid map)
  {
    _map = map; 
  }

  void addIndex(unsigned int idx)
  {
    _indices.push_back(idx); 
  }


  geometry_msgs::Point32 getCentroid(void) const
  {
    geometry_msgs::Point32 centroid; 

    for(auto i=0 ; i< _indices.size() ; i++)
    {
      std::array<unsigned int, 2> coord = thi::MapOperations::getCoordFromIndex(_map, _indices[i]); 
      centroid.x      += coord[0]; 
      centroid.y      += coord[1]; 
    }
    centroid.x = centroid.x / _indices.size() * _map.info.resolution; 
    centroid.y = centroid.y / _indices.size() * _map.info.resolution;   


    return centroid; 
  }

  


private:
  std::vector<unsigned int> _indices; 
  nav_msgs::OccupancyGrid _map; 
  
};


}


#endif // __FRONTIER__
