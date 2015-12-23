/*
 * spi_sensor_read_controller.h
 *
 *  Created on: 22 Dec 2015
 *      Author: vahid
 */

#ifndef SR_RONEX_SR_RONEX_CONTROLLERS_SPI_SENSOR_READ_CONTROLLER_H_
#define SR_RONEX_SR_RONEX_CONTROLLERS_SPI_SENSOR_READ_CONTROLLER_H_

#include <ros/node_handle.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <utility>
#include <vector>

#include "sr_ronex_controllers/spi_base_controller.hpp"
#include <sr_ronex_msgs/SPI.h>

#include <dynamic_reconfigure/server.h>
#include "sr_ronex_drivers/SPIConfig.h"
#include "std_msgs/Float64.h"

namespace ronex
{
class SPISensorReadController
  : public SPIBaseController
{
public:
  virtual bool init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n);

  bool command_srv_cb(sr_ronex_msgs::SPI::Request &req,
                       sr_ronex_msgs::SPI::Response &res,
                       size_t spi_out_index);

  void dynamic_reconfigure_cb(sr_ronex_drivers::SPIConfig &config, uint32_t level);
  void update(const ros::Time&, const ros::Duration&);

private:
  static const size_t spi_channel_;
  static const size_t sensor_message_length_;
  static const size_t spi_mode_;

  std_msgs::Float64 sensor_msg_;
  std::vector<ros::ServiceServer> command_srv_;
  ros::Publisher sensor_data_publisher_;
  // vector containing one command per spi output.
  // Some parameters of these commands are updated through the dynamic reconfigure interface
  // The data packet is updated from the service.
  std::vector<SplittedSPICommand> standard_commands_;

  /// Dynamic reconfigure server for setting the parameters of the driver
  boost::scoped_ptr<dynamic_reconfigure::Server<sr_ronex_drivers::SPIConfig> > dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<sr_ronex_drivers::SPIConfig>::CallbackType function_cb_;
  bool first_run_;
  // Instantiating the services / dynamic reconfigure callbacks etc..
  void post_init_();
};
}  // namespace ronex

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* SR_RONEX_SR_RONEX_CONTROLLERS_SPI_SENSOR_READ_CONTROLLER_H_ */
