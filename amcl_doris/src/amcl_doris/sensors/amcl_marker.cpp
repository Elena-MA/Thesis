/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL Marler routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_marker.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <vector>

#include "amcl_doris/sensors/amcl_marker.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLMarker::AMCLMarker(int simulation) : AMCLSensor()

{

  this->simulation=simulation;
  this->LoadCameraInfo();


  return;
}

AMCLMarker::~AMCLMarker()
{
  if(temp_obs.size()>0){
      temp_obs.clear();

  }
}


void
AMCLMarker::SetModelLikelihoodField(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double landa,
                                   double marker_coeff)
{
  this->model_type = MARKER_MODEL_LIKELIHOOD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->landa=landa;
  this->marker_coeff=marker_coeff;
}




////////////////////////////////////////////////////////////////////////////////
/**
 * @brief AMCLMarker::UpdateSensor apply camera observation model
 * @param pf
 * @param data
 * @return
 */
bool AMCLMarker::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  // Apply the camera sensor model
 if(this->model_type == MARKER_MODEL_LIKELIHOOD)
     cout<<"llego"<<endl;
    pf_update_sensor(pf, (pf_sensor_model_fn_t) ObservationLikelihood, data);
  return true;
}



/**
 * @brief AMCLMarker::ObservationLikelihood weighting samples with visual information
 * @param data: detected markers
 * @param set: set of samples
 * @return total weight of sample set
 */
double AMCLMarker::ObservationLikelihood(AMCLMarkerData *data, pf_sample_set_t* set)
{
  //Initializing parameters
  AMCLMarker *self;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;
  double total_weight;
  double pz,p;
  std::vector<float> z;
  self = (AMCLMarker*) data->sensor;
  std::vector<Marcador> observation=data->markers_obs;
  total_weight = 0.0;
  int i;
  std::vector<Marcador> detected_from_map;
  float gaussian_norm=1/(sqrt(2*M_PI*self->sigma_hit*self->sigma_hit));
  //Find detected markers in the map
  for(int k=0;k<observation.size();k++){
        for (int j=0; j<self->map.size();j++){

            if(self->map[j].getMarkerID()==observation[k].getMarkerID() && self->map[j].getSectorID()==observation[k].getSectorID() && self->map[j].getMapID()==observation[k].getMapID()){
                detected_from_map.push_back(self->map[j]);
                cout<<observation[k].getMarkerID()<<endl;
            }

        }
  }
  cout<<"llego"<<endl;
  cout<<detected_from_map.size()<<endl;
  for (i=0;i< set->sample_count; i++){
      sample=set-> samples + i;
      pose = sample->pose;
      p=1.0;

      //Initialize parameters
      double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
      //sqrt(2) beacuse of the normalization with height and width of image.
      double z_rand_mult=1.0/sqrt(2);


      geometry_msgs::Pose sample_pose;
      tf::Quaternion quat;
      geometry_msgs::Quaternion quat_msg;
      sample_pose.position.x=pose.v[0];
      sample_pose.position.y=pose.v[1];
      sample_pose.position.z=0.0;

      pose.v[2]=fmod(pose.v[2],2*M_PI);
      if (pose.v[2]<0){
          pose.v[2]=pose.v[2]+2*M_PI;
      }
      quat.setRPY(0,0,pose.v[2]);
      tf::quaternionTFToMsg(quat,quat_msg);
      sample_pose.orientation=quat_msg;

      for (int j=0;j<observation.size();j++){

          //Calculate projection of marker corners
           std::vector<geometry_msgs::Point> relative_to_cam=self->CalculateRelativePose(detected_from_map[j],sample_pose);
           std::vector<cv::Point2d> projection;

           //Porject points to image
           if (self->simulation == 1){
                projection=self->projectPoints(relative_to_cam);
           }
           if(self->simulation == 0){
               cout<<"aqui1"<<endl;
               std::vector<cv::Point3f>rel;
               for (int k=0; k< relative_to_cam.size(); k++){
                    cv::Point3d Coord;
                    Coord.x=relative_to_cam[k].x;
                    Coord.y=relative_to_cam[k].y;
                    Coord.z=relative_to_cam[k].z;
                    rel.push_back(Coord);
               }
               std::vector<cv::Point2f> imagePoints;
               cv::Mat rvec(3,1,cv::DataType<double>::type);
               cv::Mat tvec(3,1,cv::DataType<double>::type);
               rvec.at<double>(0)=0.0;
               rvec.at<double>(1)=0.0;
               rvec.at<double>(2)=0.0;
               tvec.at<double>(0)=0.0;
               tvec.at<double>(1)=0.0;
               tvec.at<double>(2)=0.0;

                cv::omnidir::projectPoints(rel,imagePoints,rvec,tvec,self->camMatrix,self->xi,self->distCoeff);
                for(int i=0; i<imagePoints.size();i++){
                    projection.push_back( cv::Point2d( (double)imagePoints[i].x, (double)imagePoints[i].y  ) );
                }
           }

            //Caculate error
            std::vector<cv::Point2f> Puntos=observation[j].getMarkerPoints();
            z=self->calculateError(observation[j].getMarkerPoints(),projection);
            float ztot=std::accumulate(z.begin(), z.end(), 0.0);

            //Calculate weight
            pz=0.0;
            pz+=self->landa*exp(-self->landa*ztot);
            p+=pz*pz*pz;

      }

      //Updating particle
      sample->weight *= p;
      total_weight += sample->weight;


  }
  return(total_weight);
  }


/**
 * @brief AMCLMarker::calculateError
 * @param projection_detected : corners of detected markers
 * @param projection_map: reference corners present in the map
 * @return error between detected and the projection of saved markers
 */
std::vector<float> AMCLMarker::calculateError(std::vector<cv::Point2f> projection_detected, std::vector<cv::Point2d> projection_map){
    std::vector<float> errorv;
    for (int i=0;i<4;i++){
        float errorx,errory;
        float error=0.0;
        errorx=abs(projection_map[i].x-projection_detected[i].x)/image_width;
        errory=abs(projection_map[i].y-projection_detected[i].y)/image_height;
        error+=sqrt((errorx*errorx)+(errory*errory));
        errorv.push_back(error);
    }
    return errorv;

}

/**
 * @brief AMCLMarker::projectPoints
 * @param cam_center_coord
 * @return projection of the given marker's corners into the camera image.
 */
std::vector<cv::Point2d> AMCLMarker::projectPoints(std::vector<geometry_msgs::Point> cam_center_coord){
   geometry_msgs::PointStamped cam_center_coord_st,cam_trans_coord_st;
    std::vector<cv::Point2d> Pixels;
    for (int i=0;i<cam_center_coord.size();i++){
         cam_center_coord_st.point=cam_center_coord[i];
    float angulo;
    angulo = atan2(double(cam_center_coord[i].x),double(cam_center_coord[i].z));
    angulo=fmod(angulo,2*M_PI);
    if (angulo<0){
            angulo=angulo+(2*M_PI);
        }
    cv::Point2d Pixel,offset;
    offset.x=image_width/num_cam;
    offset.y=0;
    cv::Point3d Coord;
    if (angulo>M_PI and angulo<5.2333333){
           //camera2
            tf2::doTransform(cam_center_coord_st,cam_trans_coord_st,tf_cameras[1]);
            Coord.x=cam_trans_coord_st.point.x;
            Coord.y=cam_trans_coord_st.point.y;
            Coord.z=cam_trans_coord_st.point.z;
            Pixel=this->pin_model.project3dToPixel(Coord);
            Pixel=Pixel-offset;

        }else{
            //CAM3
            if(angulo>M_PI/3 and angulo<M_PI){
                tf2::doTransform(cam_center_coord_st,cam_trans_coord_st,tf_cameras[2]);
                Coord.x=cam_trans_coord_st.point.x;
                Coord.y=cam_trans_coord_st.point.y;
                Coord.z=cam_trans_coord_st.point.z;
                Pixel=this->pin_model.project3dToPixel(Coord);
                Pixel=Pixel+offset;
                }else{//CAM1
                tf2::doTransform(cam_center_coord_st,cam_trans_coord_st,tf_cameras[0]);
                Coord.x=cam_trans_coord_st.point.x;
                Coord.y=cam_trans_coord_st.point.y;
                Coord.z=cam_trans_coord_st.point.z;
                Pixel=this->pin_model.project3dToPixel(Coord);
                }
        }
     Pixels.push_back(Pixel);
        }
    return Pixels;

}

/**
 * @brief AMCLMarker::LoadCameraInfo loading camera parameters
 */
void AMCLMarker::LoadCameraInfo(void){

    //Simulated Camera Parameters
    sensor_msgs::CameraInfo cam_inf_ed;
    cam_inf_ed.header.frame_id="Cam1";
    cam_inf_ed.height=679;
    cam_inf_ed.width=604;
    cam_inf_ed.distortion_model="plumb_bob";
    double Da[5]={-0.2601958609577983, 0.05505240192232372, 0.0, -0.0045449850126361765, 0.0};
    boost::array<double, 9ul> K={ {174.746839097, 0.0, 906.0, 0.0, 174.746839097, 339.5, 0.0, 0.0, 1.0} } ;
    boost::array<double, 9ul> R={ {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0} };
    boost::array<double, 12ul> P={ {174.64077512103418, 0.0, 906.0, 0.0, 0.0, 174.64077512103418, 339.5, 0.0, 0.0, 0.0, 1.0, 0.0} };
    std::vector<double> D(Da,Da +(sizeof(Da)/sizeof(Da[0])));
    cam_inf_ed.D=D;
    cam_inf_ed.K=K;
    cam_inf_ed.R=R;
    cam_inf_ed.P=P;
    cam_inf_ed.binning_x=0.0;
    cam_inf_ed.binning_y=0.0;
    cam_inf_ed.roi.height=0;
    cam_inf_ed.roi.width=0;

    //Real omnicamera parameters
    camMatrix = cv::Mat(3, 3, CV_32F);
    camMatrix.at<float>(0, 0) = 8.5101024687735935e+02;
    camMatrix.at<float>(0, 1) = -2.2255059056366439e-01;
    camMatrix.at<float>(0, 2) = 6.5571465382877625e+02;
    camMatrix.at<float>(1, 0) = 0.0;
    camMatrix.at<float>(1, 1) = 8.5170243585411265e+02;
    camMatrix.at<float>(1, 2) = 5.1216084358475405e+02;
    camMatrix.at<float>(2, 0) = 0.0;
    camMatrix.at<float>(2, 1) = 0.0;
    camMatrix.at<float>(2, 2) = 1.0;

    distCoeff = cv::Mat(4, 1, CV_32F);
    distCoeff.at<float>(0, 0) = -4.2648301140911193e-01;
    distCoeff.at<float>(1, 0) = 3.1105618959437248e-01;
    distCoeff.at<float>(2, 0) = -1.3775384616268102e-02;
    distCoeff.at<float>(3, 0) = -1.9560559208606078e-03;

    xi=1.5861076761699640e+00;

    this->pin_model.fromCameraInfo(cam_inf_ed);

}

/**
 * @brief AMCLMarker::CalculateRelativePose
 * @param Marca : object of class marcador
 * @param CamaraMundo : pose of camera in world frame
 * @return pose of corners in camera frame.
 */
std::vector<geometry_msgs::Point> AMCLMarker::CalculateRelativePose (Marcador Marca, geometry_msgs::Pose CamaraMundo){
    //Pose CAM;
    tf::Transform MundTrob, invMundTrob,RobTCam,invRobotTCam;
    tf::Quaternion RotCam;
    //From Robot base to camera
   if (this->simulation==1){
     RotCam.setRPY(-M_PI/2,0,-M_PI/2);
     RobTCam.setOrigin(tf::Vector3(0,0,1.3925));
   }
    if (this->simulation == 0)

    {
        RotCam.setRPY(0,0,-M_PI/2+M_PI);
        RobTCam.setOrigin(tf::Vector3(-0.26,0,1.415));
    }
    RobTCam.setRotation(RotCam);
    tf::Quaternion QMundRCam (CamaraMundo.orientation.x,CamaraMundo.orientation.y,CamaraMundo.orientation.z,CamaraMundo.orientation.w);
    tf::Vector3 Trasl1 (CamaraMundo.position.x,CamaraMundo.position.y,CamaraMundo.position.z);

    //From World to Robot
    MundTrob.setRotation(QMundRCam);
    MundTrob.setOrigin(Trasl1);

    //Inverse the transformation--> inverse form world to camera.
    invRobotTCam=RobTCam.inverse();
    invMundTrob = MundTrob.inverse();
    geometry_msgs::TransformStamped MundTrobSt, RobotTCamSt;
    MundTrobSt.header.frame_id="ground_plane__link";
    MundTrobSt.child_frame_id="EstimatedPose";
    RobotTCamSt.header.frame_id="EstimatedPose";
    RobotTCamSt.child_frame_id="camera_link";
    transformTFToMsg(MundTrob,MundTrobSt.transform);
    transformTFToMsg(RobTCam,RobotTCamSt.transform);

    //Pose Transformation
    geometry_msgs::TransformStamped invMundTrobStamped,invRobotTCamSt;
    transformTFToMsg(invMundTrob,invMundTrobStamped.transform);
    transformTFToMsg(invRobotTCam,invRobotTCamSt.transform);
    std::vector<geometry_msgs::Point> RelativaCorners,PoseWorld;
    PoseWorld=Marca.getPoseWorld();
    for (int i=0;i<4;i++){
            geometry_msgs::PointStamped CornerRelPose,Inter,WorldPose;
            WorldPose.point=PoseWorld[i];
             tf2::doTransform(WorldPose,Inter,invMundTrobStamped);
             tf2::doTransform(Inter,CornerRelPose,invRobotTCamSt);
             RelativaCorners.push_back(CornerRelPose.point);

        }
    return RelativaCorners;
}


