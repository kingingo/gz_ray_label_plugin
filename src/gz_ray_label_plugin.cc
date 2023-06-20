#include <iostream>
#include <math.h>
#include <gazebo/common/Plugin.hh>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <gazebo/rendering/DynamicLines.hh>
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include <gazebo/rendering/rendering.hh>
#include <ignition/transport/Node.hh>
#include "gz_ray_label_plugin/LabelPoint.h"
#include "gz_ray_label_plugin/LabelPoints.h"
#include <visualization_msgs/Marker.h>
#include <sstream>
#define MAKE_STRING(tokens) /****************/ \
    static_cast<std::ostringstream&>(          \
        std::ostringstream().flush() << tokens \
    ).str()


namespace gazebo
{

class GZRayLabelPlugin : public WorldPlugin
{
    private: transport::NodePtr node;
    private: physics::WorldPtr world;
    private: ros::Subscriber subscriber;
    private: ros::NodeHandle nh;
    private: ros::Publisher publisher;
    private: ros::Publisher marker_publisher;

    public: GZRayLabelPlugin() : WorldPlugin(){
    }

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf){
        if (!ros::isInitialized()){
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
        }

        node = transport::NodePtr(new transport::Node());
        world = _parent;

        #if GAZEBO_MAJOR_VERSION >= 8
            node->Init(world->Name());
        #else
            node->Init(world->GetName());
        #endif

        subscriber = nh.subscribe("/ray/points", 10, &GZRayLabelPlugin::onSubscribe, this);
        publisher = nh.advertise<gz_ray_label_plugin::LabelPoints>("/ray/labeled/points", 10);
        marker_publisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        log("Ray label plugin has been loaded");
    }

    public: void log(const std::string msg){
        std::cout << "\033[32m[gz-ray] " << msg.c_str() << "\033[0m" << std::endl;
    }

    public: void onSubscribe(const gz_ray_label_plugin::LabelPoints& data){
        log("got a message :)");
        std::string not_found = "None";
        double dist;
        std::string entityName;
        gz_ray_label_plugin::LabelPoints _data;
        gz_ray_label_plugin::LabelPoint _point;
        gazebo::physics::RayShapePtr ray;
        gazebo::physics::PhysicsEnginePtr engine;
        ignition::math::Vector3d start, end, scale;
        int length = data.points.size();
        bool showMarker = false;

        // RVIZ Ray visualization 
        visualization_msgs::Marker line;
        geometry_msgs::Point line_end, line_start;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.header.frame_id = "map";
        line.ns = "ray_ns";
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.id = 0;
        line.scale.x = 0.01;
        line.scale.y = 0.01;
        line.scale.z = 0.01;
        //color red
        line.color.r = 1.0;
        line.color.a = 1.0;
        

        #if GAZEBO_MAJOR_VERSION >= 8
            engine = world->Physics();
        #else
            engine = world->GetPhysicsEngine();
        #endif
        engine->InitForThread();
        ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>( 
                engine->CreateShape("ray", gazebo::physics::CollisionPtr())
            );

        _data.points.reserve(length);
        log(MAKE_STRING("points length:" << length));
        
        int c = 1;
        int f = 0;
        int d = 0;
        //ros::Rate r(2000);
        length = data.points.size();
        d = length / 100;
        c = 1;
        for(auto it = data.points.begin(); it != data.points.end(); it++){
            if(c % d == 0){
                log(MAKE_STRING("found " << f << " Percent completed "  << ceil(c * 100.0 / length) << "%"));
            }
            _point.x = it->x;
            _point.y = it->y;
            _point.z = it->z;
            _point.index = it->index;

            scale.X(0);
            scale.Y(0);
            scale.Z(it->z - data.start_z);

            start.X(it->x);
            start.Y(it->y);
            start.Z(data.start_z);

            end.X(it->x + scale.X() * data.scaling);
            end.Y(it->y + scale.Y() * data.scaling);
            end.Z(it->z + scale.Z() * data.scaling);

            ray->SetPoints(start, end);
            ray->GetIntersection(dist, entityName);

            if (!entityName.empty()){
                f++;
                _point.dist = dist;
                _point.entityName = entityName;
            }else{
                _point.entityName = not_found;
            }
            _data.points.push_back(_point);
            c++;
            
            line_start.x = it->x;
            line_start.y = it->y;
            line_start.z = data.start_z;

            line_end.x = it->x + scale.X() * data.scaling;
            line_end.y = it->y + scale.Y() * data.scaling;
            line_end.z = it->z + scale.Z() * data.scaling;
            
            line.points.clear();
            line.points.push_back(line_start);
            line.points.push_back(line_end);

            if(showMarker){
                marker_publisher.publish(line);
                //r.sleep();
            }
        }
        ray.reset();
        publisher.publish(_data);
        log(MAKE_STRING("done scanning total found " << f));
        _data.points.clear();
    }
};
GZ_REGISTER_WORLD_PLUGIN(GZRayLabelPlugin)
}