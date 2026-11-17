#include "crazyflie_webots/webots_driver/webots_crazyflie_driver.hpp"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <iomanip>

WebotsCrazyflieDriver::WebotsCrazyflieDriver(int id, int webots_port, bool webots_use_tcp, const std::string &webots_tcp_ip)
      : WebotsRobotDriver("cf" + std::to_string(id), webots_port, webots_use_tcp, webots_tcp_ip) 
      , m_id(id)
{
    // Initialize motors
    m_m1_motor = wb_robot_get_device("m1_motor");
    wb_motor_set_position(m_m1_motor, INFINITY);
    m_m2_motor = wb_robot_get_device("m2_motor");
    wb_motor_set_position(m_m2_motor, INFINITY);
    m_m3_motor = wb_robot_get_device("m3_motor");
    wb_motor_set_position(m_m3_motor, INFINITY);
    m_m4_motor = wb_robot_get_device("m4_motor");
    wb_motor_set_position(m_m4_motor, INFINITY);
  
    // The supervisor supplies velocity, so propulsion must first balance the
    // model's weight. The old fixed 48 rad/s produced only 0.369 N for a 0.491 N
    // vehicle, making each FullState/hold mode change lose height.
    auto physics_field=wb_supervisor_node_get_base_node_field(m_robot_node,"physics");
    auto physics=physics_field ? wb_supervisor_field_get_sf_node(physics_field) : nullptr;
    auto mass_field=physics ? wb_supervisor_node_get_field(physics,"mass") : nullptr;
    if(!mass_field) throw std::runtime_error("Crazyflie model requires explicit Physics.mass");
    const double mass=wb_supervisor_field_get_sf_float(mass_field);
    double gravity=9.81;
    auto children=wb_supervisor_node_get_field(wb_supervisor_node_get_root(),"children");
    for(int i=0;i<wb_supervisor_field_get_count(children);++i){
        auto node=wb_supervisor_field_get_mf_node(children,i);
        if(std::string(wb_supervisor_node_get_type_name(node))=="WorldInfo"){
            auto field=wb_supervisor_node_get_field(node,"gravity");
            if(field) gravity=wb_supervisor_field_get_sf_float(field);
        }
    }
    double thrust_coefficient=0;
    for(const auto* name : {"M1","M2","M3","M4"}){
        auto propeller=wb_supervisor_node_get_from_proto_def(m_robot_node,name);
        auto field=propeller ? wb_supervisor_node_get_field(propeller,"thrustConstants") : nullptr;
        if(!field) throw std::runtime_error("Crazyflie model requires M1..M4 propellers");
        const double* coefficient=wb_supervisor_field_get_sf_vec2f(field);
        if(std::abs(coefficient[1])>1e-12) throw std::runtime_error("Unsupported linear propeller coefficient");
        thrust_coefficient+=std::abs(coefficient[0]);
    }
    m_hover_motor_speed=ds_motion::neutral_motor_speed(mass,gravity,thrust_coefficient);
    for(int i=0;i<wb_supervisor_field_get_count(children);++i) {
        auto node=wb_supervisor_field_get_mf_node(children,i);
        if(wb_supervisor_node_get_type(node)!=WB_NODE_ROBOT) continue;
        auto name_field=wb_supervisor_node_get_base_node_field(node,"name");
        if(!name_field) continue;
        std::string name=wb_supervisor_field_get_sf_string(name_field);
        if(name.size()>2 && name.rfind("cf",0)==0) {
            try { m_robots.emplace(std::stoi(name.substr(2)),node); } catch(...) {}
        }
    }
    wb_supervisor_node_enable_contact_points_tracking(m_robot_node, static_cast<int>(get_time_step()), true);

    // Initialize sensors
    m_gps = wb_robot_get_device("gps");
    wb_gps_enable(m_gps, static_cast<int>(get_time_step()));
    m_zranger = wb_robot_get_device("zrange");
    m_ranger_front = wb_robot_get_device("range_front");
    m_ranger_back = wb_robot_get_device("range_back");
    //m_ranger_up = wb_robot_get_device("range_up");
    m_ranger_left = wb_robot_get_device("range_left");
    m_ranger_right = wb_robot_get_device("range_right");

    Eigen::Vector3d pos = get_robot_pose().translation();
    m_target = {pos.x(), pos.y(), pos.z()};
    
    for(auto tag : {m_ranger_front,m_ranger_back,m_ranger_left,m_ranger_right,m_zranger})
        wb_distance_sensor_enable(tag, static_cast<int>(get_time_step()));
    m_past_time = wb_robot_get_time();
    m_sim_time.store(m_past_time);
    m_previous_position=pos;
    ds_motion::Pose initial; initial.position=pos; initial.orientation=Eigen::Quaterniond(get_robot_pose().linear());
    motion.tick(m_past_time,initial);
    auto synchronization=wb_supervisor_node_get_field(m_robot_node,"synchronization");
    if(synchronization) wb_supervisor_field_set_sf_bool(synchronization,false);
}


namespace {
double metres(WbDeviceTag tag) {
    const double value=wb_distance_sensor_get_value(tag);
    const int n=wb_distance_sensor_get_lookup_table_size(tag);
    const double* table=wb_distance_sensor_get_lookup_table(tag);
    if(n<2 || !table || !std::isfinite(value)) return INFINITY;
    for(int i=1;i<n;++i) {
        const double a=table[(i-1)*3+1], b=table[i*3+1];
        if(value>=std::min(a,b) && value<=std::max(a,b) && a!=b) {
            double distance=table[(i-1)*3]+(value-a)/(b-a)*(table[i*3]-table[(i-1)*3]);
            return distance>=table[(n-1)*3] ? INFINITY : distance;
        }
    }
    return INFINITY;
}
}
bool
WebotsCrazyflieDriver::step()
{
    if(!WebotsRobotDriver::step()) return false;
    const double now=wb_robot_get_time();
    const double dt=now-m_past_time;
    const auto pose=get_robot_pose();
    ds_motion::Pose measured; measured.position=pose.translation(); measured.orientation=Eigen::Quaterniond(pose.linear());
    auto command=motion.tick(now,measured);
    Eigen::Vector3d bias;
    bool registered=false,clear=false;
    double schedule=-1;
    ds_motion::TrialGrid grid;
    {
        std::lock_guard<std::mutex> lock(m_disturbance_mutex);
        bias=m_disturbance.at(now); registered=m_trial_registered;
        schedule=m_schedule_epoch;
        grid=m_trial_grid;
        clear=m_clear_trial; m_clear_trial=false;
    }
    if(clear) {
        m_contacts.clear(); m_min_distance=INFINITY; m_min_peer=0; m_grid_conflicts.clear();
        for(const auto& robot:m_robots)
            wb_supervisor_node_enable_pose_tracking(robot.second,static_cast<int>(get_time_step()),nullptr);
    }
    Eigen::Vector3d perturbed=command.linear+bias;
    if(perturbed.norm()>3) perturbed*=3/perturbed.norm();
    const double vel[6]={perturbed.x(),perturbed.y(),perturbed.z(),command.angular.x(),command.angular.y(),command.angular.z()};
    wb_supervisor_node_set_velocity(m_robot_node,vel);
    int contact_count=0;
    auto points=wb_supervisor_node_get_contact_points(m_robot_node,true,&contact_count);
    for(int i=0;i<contact_count;++i) {
        auto node=wb_supervisor_node_get_from_id(points[i].node_id);
        for(int depth=0;node && depth<32;++depth) {
            bool found=false;
            for(const auto& robot:m_robots) if(wb_supervisor_node_get_id(robot.second)==wb_supervisor_node_get_id(node)) {
                if(robot.first!=m_id) m_contacts.emplace(robot.first,now);
                found=true; break;
            }
            if(found) break;
            node=wb_supervisor_node_get_parent_node(node);
        }
    }
    if(registered && !clear) {
        // Cached tracked poses belong to this physics step. Count each pair once.
        const double* self=wb_supervisor_node_get_pose(m_robot_node,nullptr);
        const Eigen::Vector3d self_position(self[3],self[7],self[11]);
        const int cell=grid.cell(self_position);
        for(const auto& robot:m_robots) {
            const int peer=robot.first;
            if(peer<=m_id) continue;
            const double* other=wb_supervisor_node_get_pose(robot.second,nullptr);
            const Eigen::Vector3d peer_position(other[3],other[7],other[11]);
            const double distance=(self_position-peer_position).norm();
            if(distance<m_min_distance) {m_min_distance=distance;m_min_time=now;m_min_peer=peer;}
            if(cell>=0 && cell==grid.cell(peer_position) && !m_grid_conflicts.count(peer)) {
                std::ostringstream conflict;
                conflict<<std::setprecision(17)<<"{\"time\":"<<now<<",\"cell\":"<<cell
                    <<",\"drone_id\":"<<m_id<<",\"peer_id\":"<<peer<<",\"position\":["
                    <<self_position.x()<<","<<self_position.y()<<","<<self_position.z()
                    <<"],\"peer_position\":["<<peer_position.x()<<","<<peer_position.y()<<","<<peer_position.z()<<"]}";
                m_grid_conflicts.emplace(peer,conflict.str());
            }
        }
    }
    std::ostringstream experiment;
    experiment<<std::setprecision(17)<<"{\"sim_time\":"<<now<<",\"schedule_epoch\":"<<schedule
        <<",\"bias\":["<<bias.x()<<","<<bias.y()<<","<<bias.z()<<"],\"position\":["
        <<measured.position.x()<<","<<measured.position.y()<<","<<measured.position.z()<<"],\"minimum_distance\":";
    if(std::isfinite(m_min_distance)) experiment<<m_min_distance; else experiment<<"null";
    experiment<<",\"minimum_distance_at\":"<<m_min_time
        <<",\"minimum_distance_pair\":["<<m_id<<","<<m_min_peer<<"],\"grid_conflicts\":[";
    bool comma=false;
    for(const auto& conflict:m_grid_conflicts) {
        if(comma) experiment<<",";
        comma=true;experiment<<conflict.second;
    }
    experiment<<"],\"contacts\":[";
    comma=false;
    for(const auto& contact:m_contacts) {
        if(comma) experiment<<",";
        comma=true;
        experiment<<"{\"other_id\":"<<contact.first<<",\"time\":"<<contact.second<<"}";
    }
    experiment<<"]}";
    {
        std::lock_guard<std::mutex> lock(m_sensor_mutex);
        if(dt>0) m_velocity=(measured.position-m_previous_position)/dt;
        m_previous_position=measured.position;
        m_ranges={metres(m_ranger_back),metres(m_ranger_right),metres(m_ranger_front),metres(m_ranger_left),metres(m_zranger),INFINITY};
        m_experiment_status=experiment.str();
    }
    m_sim_time.store(now);
    const double speed=measured.position.z()>0.025 || command.linear.z()>0 ? m_hover_motor_speed : 0;
    wb_motor_set_velocity(m_m1_motor,-speed); wb_motor_set_velocity(m_m2_motor,speed);
    wb_motor_set_velocity(m_m3_motor,-speed); wb_motor_set_velocity(m_m4_motor,speed);
    m_past_time=now;
    return true;
}

void WebotsCrazyflieDriver::set_disturbance(const std::vector<double>& input) {
    std::lock_guard<std::mutex> lock(m_disturbance_mutex);
    if(input.empty()) {m_disturbance.cancel();m_trial_registered=false;m_schedule_epoch=-1;return;}
    // epoch, rows/cols/layers, origin[3], step[3], count, then six values per pulse.
    if(input.size()<11 || !std::isfinite(input[0]) || input[0]<simulation_time())
        throw std::invalid_argument("Invalid event schedule epoch");
    for(int i=1;i<=3;++i)
        if(!std::isfinite(input[i]) || input[i]<1 || input[i]>2147483647 || input[i]!=std::floor(input[i]))
            throw std::invalid_argument("Invalid observation grid dimensions");
    if(!std::isfinite(input[10]) || input[10]<0 || input[10]!=std::floor(input[10])
        || input[10]>(input.size()-11)/6 || input.size()!=11+6*static_cast<std::size_t>(input[10]))
        throw std::invalid_argument("Invalid event schedule length");
    ds_motion::TrialGrid grid{{input[4],input[5],input[6]}, {input[7],input[8],input[9]},
        static_cast<int>(input[1]),static_cast<int>(input[2]),static_cast<int>(input[3])};
    grid.validate();
    std::vector<ds_motion::WindPulse> pulses;
    for(std::size_t i=11;i<input.size();i+=6)
        pulses.push_back({input[i],input[i+1],input[i+2],{input[i+3],input[i+4],input[i+5]}});
    m_disturbance.arm(std::move(pulses),simulation_time());
    m_trial_grid=grid;m_schedule_epoch=input[0];m_trial_registered=true;m_clear_trial=true;
}

void
WebotsCrazyflieDriver::set_target(Eigen::Vector3d target, double yaw_deg)
{
    motion.position(target,yaw_deg*M_PI/180.0);

}

double
WebotsCrazyflieDriver::get_battery_voltage()
{
    return 4.2;    // Dummy value for battery voltage
}

double
WebotsCrazyflieDriver::get_charge_current()
{
    return 0.0;    // Dummy value for charge current
}

double
WebotsCrazyflieDriver::get_charge_state()
{
    return double(ChargeState::CHARGED); // Dummy value for charge state
} 

bool
WebotsCrazyflieDriver::can_fly()
{
    return true; // Dummy value for can fly
}

bool
WebotsCrazyflieDriver::is_flying()
{
    double z_global = get_robot_pose().translation().z();
    return (z_global > 0.05); // Consider flying if above 5 cm
}

bool
WebotsCrazyflieDriver::is_tumbled()
{
    return false; // Dummy value for is tumbled
}

double
WebotsCrazyflieDriver::get_range_front()
{ return ranges()[2];
}

double
WebotsCrazyflieDriver::get_range_back()
{ return ranges()[0];
}

double
WebotsCrazyflieDriver::get_range_up()
{ return ranges()[5];
}   

double
WebotsCrazyflieDriver::get_range_left()
{ return ranges()[3];
}   

double
WebotsCrazyflieDriver::get_range_right()
{ return ranges()[1];
}   

double
WebotsCrazyflieDriver::get_range_zrange()
{ return ranges()[4];
}
