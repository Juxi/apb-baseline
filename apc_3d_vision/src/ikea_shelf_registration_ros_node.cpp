/*
   Copyright 2016 Australian Centre for Robotic Vision
 */

#include <apc_3d_vision.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <shape_msgs/Mesh.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>

#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"

static const double PUBLISH_UPDATE_FREQUENCY = 10; // Publish @ 10 Hz;

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > kinect_cloud;
bool   triggerRepublish;
double max_depth;
double max_right_crop;
double max_left_crop;
tf::TransformListener *tf_listener_global;

boost::shared_ptr<tf::TransformListener> tf_list;
bool mannual_trigger_update_ = false;

void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    pcl::fromROSMsg(*msg, *kinect_cloud);

    pcl::PointCloud<pcl::PointXYZ>::iterator itr = (*kinect_cloud).begin();

    for (; itr != (*kinect_cloud).end(); ++itr) {
        if (itr->z > max_depth) {
            itr->z = NAN;
        }

        if (itr->x > max_right_crop) {
            itr->x = NAN;
        }

        if (itr->x < max_left_crop) {
            // NOTE make sure max_left_crop is negative
            itr->x = NAN;
        }
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*kinect_cloud, *kinect_cloud, indices);
}

void republishPrealignedShelfCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data == true) {
        triggerRepublish = true;
        ROS_INFO("Republishing Shelf");
    }
}

// Publish shelf tf's based on location of the shelf in the scene


bool transformPose(const geometry_msgs::PoseStamped& input_pose,
                   std::string                       target_frame_id,
                   geometry_msgs::PoseStamped       *transformed_pose) {
    bool tf_success = tf_listener_global->waitForTransform(
        target_frame_id, input_pose.header.frame_id, ros::Time(0),
        ros::Duration(1.0));

    if (tf_success) {
        tf_listener_global->getLatestCommonTime(
            input_pose.header.frame_id, target_frame_id,
            transformed_pose->header.stamp, NULL);

        tf_listener_global->transformPose(target_frame_id, input_pose,
                                          *transformed_pose);
        return true;
    } else {
        ROS_INFO("Can't Find Transform from %s to %s!",
                 input_pose.header.frame_id.c_str(), target_frame_id.c_str());
        return false;
    }
}

int main(int argc, char **argv) {
    triggerRepublish = true; // Publish shelf on first alignment always
    ros::init(argc, argv, "shelf_registration_ros_node");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    tf_listener_global = new tf::TransformListener();
    Apc3dVision apc_vis;
    kinect_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Publish on the kinect_toggle topic
    ros::Publisher kinectTogglePub =
        nh.advertise<std_msgs::String>("/kinect_toggle", 1000);

    // Wait for the kinect toggle subscriber node to come online
    while (kinectTogglePub.getNumSubscribers() == 0) {
        printf("Waiting for kinect toggle node to come online...\n");
        ros::Duration(0.5).sleep();
    }

    ros::Rate kinect_toggle_publish_rate(PUBLISH_UPDATE_FREQUENCY); // 10 Hz
    std_msgs::String kinect_toggle_msg;
    kinect_toggle_msg.data = "on";
    kinectTogglePub.publish(kinect_toggle_msg);

    tf::TransformListener tf_listener;
    tf::StampedTransform  my_transform;

    double transformationEpsilon;
    double stepSize;
    double resolution;
    std::vector<float> initialShelfTransform;

    // get params local to empty_shelf_publisher node
    local_nh.param("mytransformationEpsilon", transformationEpsilon, 0.001);
    local_nh.param("mystepSize",              stepSize,              0.02);
    local_nh.param("myresolution",            resolution,            1.0);
    local_nh.param("max_depth",               max_depth,             3.0);
    local_nh.param("max_right_crop",          max_right_crop,        3.0);
    local_nh.param("max_left_crop",           max_left_crop,         3.0);
    local_nh.getParam("InitialShelfTransform", initialShelfTransform);
    std::string pcd_path;
    local_nh.getParam("pcd_path",              pcd_path);

    std::string topicPoints   = "/kinect2/qhd/points";
    ros::Subscriber subPoints = nh.subscribe(topicPoints, 10, pointsCallback);

    ros::Subscriber republishShelf = nh.subscribe(
        "/republish_prealigned_shelf", 10, republishPrealignedShelfCallback);

    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(
        "registered_empty_shelf", 1);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > empty_shelf;
    empty_shelf.reset(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > shelf;
    shelf.reset(new pcl::PointCloud<pcl::PointXYZ>);

    std::string savePath = getenv("HOME");
    savePath = savePath + pcd_path;
    std::string fileName = savePath;
    fileName = fileName + "empty_shelf_cloud.pcd";
    std::string shelf_fn = savePath;
    shelf_fn = shelf_fn + "aligned_pod.pcd";

    apc_vis.load_pcd_file(fileName, empty_shelf);
    apc_vis.load_pcd_file(shelf_fn, shelf);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > registered_empty_shelf;
    registered_empty_shelf.reset(new pcl::PointCloud<pcl::PointXYZ>);

    Apc3dVision::ndt_params_t params(empty_shelf, kinect_cloud);

    params.output_cloud               = registered_empty_shelf;
    params.input_cloud_leaf_size      = 0.05;
    params.is_downsample_input_cloud  = true;
    params.target_cloud_leaf_size     = 0.05;
    params.is_downsample_target_cloud = false;
    params.transformationEpsilon      = transformationEpsilon;
    params.stepSize                   = stepSize;
    params.resolution                 = resolution;
    params.maxIterations              = 100;
    params.verbose                    = false;

    bool isShelfRegistered = false;
    bool isNotRegistered   = true;

    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);

    // ---------------- Shelf CAD Model ---------------------------------

    ros::Publisher planning_scene_publisher =
        nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = "/base";
    collision_object.header.stamp    = ros::Time::now();
    collision_object.id              = "shelf";
    shapes::Mesh *m = shapes::createMeshFromResource(
        "package://apc_3d_vision/models_apb/pod_lowres.stl");

    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m, co_mesh_msg);

    shape_msgs::Mesh co_mesh;
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    geometry_msgs::Pose shelf_pose;
    geometry_msgs::PoseStamped shelf_pose_stamped;
    geometry_msgs::PoseStamped shelf_pose_in_base_stamped;

    Eigen::Translation3f shelf_translation(
        Eigen::Vector3f(initialShelfTransform[3], initialShelfTransform[7],
                        initialShelfTransform[11]));

    Eigen::Matrix3f shelf_rotation;
    shelf_rotation << initialShelfTransform[0], initialShelfTransform[1],
        initialShelfTransform[2], initialShelfTransform[4],
        initialShelfTransform[5], initialShelfTransform[6],
        initialShelfTransform[8], initialShelfTransform[9],
        initialShelfTransform[10];

    Eigen::Quaternionf shelf_rotation_quaternion(shelf_rotation);

    tf::TransformBroadcaster br;

    collision_object.meshes.push_back(co_mesh);

    // collision_object.mesh_poses.push_back(shelf_pose);
    collision_object.operation = moveit_msgs::CollisionObject::ADD;

    // --------------
    ros::Rate publish_rate(PUBLISH_UPDATE_FREQUENCY);

    while (nh.ok()) {
        // Perform shelf registration when you get a kinect cloud
        if (isNotRegistered && (kinect_cloud->size() > 0)) {
            isShelfRegistered = apc_vis.align_ndt(&params);
            isNotRegistered   = false;
            pcl::transformPointCloud(*shelf, *shelf, params.transform);

            // Update shelf_pose
            Eigen::Transform<float, 3, Eigen::Affine> combined =
                params.transform * shelf_translation *
                shelf_rotation_quaternion;

            shelf_pose.position.x = combined.translation()[0];
            shelf_pose.position.y = combined.translation()[1];
            shelf_pose.position.z = combined.translation()[2];
            Eigen::Quaternionf combined_quaternion(combined.rotation());
            shelf_pose.orientation.x = combined_quaternion.x();
            shelf_pose.orientation.y = combined_quaternion.y();
            shelf_pose.orientation.z = combined_quaternion.z();
            shelf_pose.orientation.w = combined_quaternion.w();

            shelf_pose_stamped.header.stamp            = ros::Time::now();
            shelf_pose_stamped.header.frame_id         = "/kinect2_link";
            shelf_pose_stamped.pose                    = shelf_pose;
            shelf_pose_in_base_stamped.header.stamp    = ros::Time::now();
            shelf_pose_in_base_stamped.header.frame_id = "/base";

            transformPose(shelf_pose_stamped, "/base",
                          &shelf_pose_in_base_stamped);

            collision_object.mesh_poses.push_back(
                shelf_pose_in_base_stamped.pose);

            // Finish updating shelf_pose

            if (isShelfRegistered) {
                std::cout << "\nLooking up transform from points to torso...\n";

                try {
                    ros::Time now = ros::Time::now();

                    bool tf_success = tf_listener.waitForTransform(
                        "/torso", "/kinect2_rgb_optical_frame", now,
                        ros::Duration(4.0));

                    if (tf_success) {
                        tf_listener.lookupTransform(
                            "/torso", "/kinect2_rgb_optical_frame", now,
                            my_transform);
                    } else {
                        ROS_WARN("Could not lookup transform.");
                    }
                } catch (tf::TransformException ex) {
                    ROS_ERROR("TransformException: %s", ex.what());
                    return false;
                } catch (...) {
                    ROS_ERROR("Unknown exception.");
                    return false;
                }
                std::cout << "\nShelf registration success.\n";
                kinect_toggle_msg.data = "off";
                kinectTogglePub.publish(kinect_toggle_msg);
            } else {
                std::cout
                    << "\nShelf registration failure.\n\nExiting now...\n";
                return 0;
            }
        } else {
            // If not registered yet
            if (!isShelfRegistered) {
                ROS_WARN("Waiting for the kinect to come online...");
            }
        }

        // Publish shelf model to collision scene after registration or when
        // told to republish
        if (!isNotRegistered && (shelf->size() > 0) && triggerRepublish) {
            // http://docs.ros.org/jade/api/moveit_msgs/html/msg/PlanningScene.html
            moveit_msgs::PlanningScene planning_scene_msg;
            planning_scene_msg.is_diff = true; // Important!!!

            geometry_msgs::TransformStamped kinect_tf_stamped_msg;
            tf::StampedTransform kinect_tf_stamped;

            collision_object.header.stamp = ros::Time::now();

            // collision_object_publisher.publish(collision_object);

            planning_scene_msg.world.collision_objects.push_back(
                collision_object);

            // NOTE Disabled publishing the shelf cad model
            planning_scene_publisher.publish(planning_scene_msg);

            triggerRepublish = false; // Stop publishing until told to again
        }

        if (!isNotRegistered && (shelf->size() > 0)) {
            Eigen::Affine3d shelf_base_tf;
            tf::poseMsgToEigen(shelf_pose, shelf_base_tf);

            // Transform base to bin_a
            Eigen::Affine3d tf_1 = Eigen::Affine3d::Identity();

            // Kiva Shelf transform from shelf model frame to bin_A
            // tf_1.translation() << -0.40, 1.77, 0.43;
            // Ikea Shelf transform from shelf model frame to bin_A
            tf_1.translation() << 0.045, 1.44, 0.39;
            Eigen::Affine3d tf_2 = Eigen::Affine3d::Identity();
            tf_2.rotate(
                Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
            Eigen::Affine3d tf_3 = Eigen::Affine3d::Identity();
            tf_3.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
            Eigen::Affine3d e_base_to_bin_a =
                shelf_base_tf * tf_1 * tf_2 * tf_3;

            tf::Transform t_base_to_bin_a;
            tf::poseEigenToTF(e_base_to_bin_a, t_base_to_bin_a);

            // Transform bin_a to other bins
            double shelf_width  = 0.351;
            double shelf_height = 0.351;

            Eigen::Affine3d tf_b = Eigen::Affine3d::Identity();
            tf_b.translation() << 0.0, - shelf_width, 0.0;
            tf_b = e_base_to_bin_a * tf_b;
            Eigen::Affine3d tf_c = Eigen::Affine3d::Identity();
            tf_c.translation() << shelf_height, 0.0, 0.0;
            tf_c = e_base_to_bin_a * tf_c;

            Eigen::Affine3d tf_d = Eigen::Affine3d::Identity();
            tf_d.translation() << shelf_height, -shelf_width, 0.0;
            tf_d = e_base_to_bin_a * tf_d;
            Eigen::Affine3d tf_e = Eigen::Affine3d::Identity();
            tf_e.translation() << 2 *shelf_height, 0.0, 0.0;
            tf_e = e_base_to_bin_a * tf_e;
            Eigen::Affine3d tf_f = Eigen::Affine3d::Identity();
            tf_f.translation() <<  2 *shelf_height, -shelf_width, 0.0;
            tf_f = e_base_to_bin_a * tf_f;

            Eigen::Affine3d tf_g = Eigen::Affine3d::Identity();
            tf_g.translation() << 3 *shelf_height, 0.0, 0.0;
            tf_g = e_base_to_bin_a * tf_g;
            Eigen::Affine3d tf_h = Eigen::Affine3d::Identity();
            tf_h.translation() << 3 *shelf_height, -shelf_width, 0.0;
            tf_h = e_base_to_bin_a * tf_h;


            // -------------------------------------------------

            // Transform from kinect2_rgb_optical_frame to torso

            Eigen::Affine3d my_transform_eigen;
            tf::transformTFToEigen(my_transform, my_transform_eigen);

            e_base_to_bin_a = my_transform_eigen * e_base_to_bin_a;
            tf::poseEigenToTF(e_base_to_bin_a, t_base_to_bin_a);

            // Find the origin of the shelf in the torso frame
            Eigen::Affine3d shelf_origin_torso;
            shelf_origin_torso = my_transform_eigen * shelf_base_tf;
            tf::Transform shelf_origin;
            tf::poseEigenToTF(shelf_origin_torso, shelf_origin);

            tf_b = my_transform_eigen * tf_b;
            tf_c = my_transform_eigen * tf_c;
            tf_d = my_transform_eigen * tf_d;
            tf_e = my_transform_eigen * tf_e;
            tf_f = my_transform_eigen * tf_f;
            tf_g = my_transform_eigen * tf_g;
            tf_h = my_transform_eigen * tf_h;

            // -------------------------------------------------

            tf::Transform t_tf_b;
            tf::poseEigenToTF(tf_b, t_tf_b);
            tf::Transform t_tf_c;
            tf::poseEigenToTF(tf_c, t_tf_c);

            tf::Transform t_tf_d;
            tf::poseEigenToTF(tf_d, t_tf_d);
            tf::Transform t_tf_e;
            tf::poseEigenToTF(tf_e, t_tf_e);
            tf::Transform t_tf_f;
            tf::poseEigenToTF(tf_f, t_tf_f);

            tf::Transform t_tf_g;
            tf::poseEigenToTF(tf_g, t_tf_g);
            tf::Transform t_tf_h;
            tf::poseEigenToTF(tf_h, t_tf_h);

            // Points are published in kinect2_rgb_optical_frame but this has
            // the same tf as kinect2_link

            br.sendTransform(tf::StampedTransform(
                                 t_base_to_bin_a, ros::Time::now(), "torso",
                                 "shelf"));
            br.sendTransform(tf::StampedTransform(
                                 shelf_origin, ros::Time::now(), "torso",
                                 "shelf_origin"));
            br.sendTransform(tf::StampedTransform(
                                 t_base_to_bin_a, ros::Time::now(), "torso",
                                 "bin_A"));
            br.sendTransform(tf::StampedTransform(t_tf_b, ros::Time::now(),
                                                  "torso", "bin_B"));
            br.sendTransform(tf::StampedTransform(t_tf_c, ros::Time::now(),
                                                  "torso", "bin_C"));

            br.sendTransform(tf::StampedTransform(t_tf_d, ros::Time::now(),
                                                  "torso", "bin_D"));
            br.sendTransform(tf::StampedTransform(t_tf_e, ros::Time::now(),
                                                  "torso", "bin_E"));
            br.sendTransform(tf::StampedTransform(t_tf_f, ros::Time::now(),
                                                  "torso", "bin_F"));

            br.sendTransform(tf::StampedTransform(t_tf_g, ros::Time::now(),
                                                  "torso", "bin_G"));
            br.sendTransform(tf::StampedTransform(t_tf_h, ros::Time::now(),
                                                  "torso", "bin_H"));
        }

        ros::spinOnce();
        publish_rate.sleep();
    }

    return 0;
}
