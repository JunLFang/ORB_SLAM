/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap),mPosevector(cv::Mat(4,1,CV_32FC1)),mMap2D(cv::Mat(1500,1500,CV_32FC1))
{

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mPosevector.at<float>(0,0) = 0.0;
    mPosevector.at<float>(1,0) = 0.0;
    mPosevector.at<float>(2,0) = 0.0;
    mPosevector.at<float>(3,0) = 1.0;
    std::cout<<mPosevector<<std::endl;
    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];


}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        //add map show
//        cv::circle(mMap2D, cv::Point2f(pos.at<float>(0)*10+750,pos.at<float>(1)*10+750),0,255);
//        cout<<cv::Point2f(pos.at<float>(0)+750,pos.at<float>(1)+750)<<endl;
//        cv::imshow("map2d",mMap2D);

        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}


void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
//
//    for(auto each_key : vpKFs)
//    {
//       // cv::imshow("each_key pose",each_key->GetPose());
//        //cout<<mPosevector<<endl;
////        mPosevector=(each_key->GetPose())*mPosevector;
////        float x3D=mPosevector.at<float>(0,0);
////        float y3D=mPosevector.at<float>(1,0);
////        float x2D=x3D+250;
////        float y2D=y3D+250;
////
////        cv::circle(mMap2D, cv::Point2f(x2D,y2D),0,255);
////        cv::imshow("map2d",mMap2D);
//
////        std::cout<<mPosevector.at<float>(0,0)<<"," <<endl;
////        std::cout<<mPosevector.at<float>(1,0)<<","  <<endl;
////        std::cout<<mPosevector.at<float>(2,0)<<","  <<endl;
////        std::cout<<mPosevector.at<float>(3,0)<<"\n"<<endl;
//    }


    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();



            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();

//            cv::Mat trans_matrix(3,3,CV_32FC1);
//            float m0[]={ cos(M_PI/4),-sin(M_PI/4),0,
//                         sin(M_PI/4),cos(M_PI/4),0,
//                       0,0,1 };
//            InitMat(trans_matrix,m0);
//            cv::Mat Ow_fit;
//            Ow_fit=trans_matrix*Ow;
//            float x3D=Ow_fit.at<float>(0,0);
//            float y3D=Ow_fit.at<float>(1,0);
//            std::cout<<x3D<<"," <<endl;
//            std::cout<<y3D<<"\n"  <<endl;
//
//            cv::circle(mMap2D, cv::Point2f(x3D*10+750,y3D*10+750),0,255);
//            cv::imshow("map2d",mMap2D);
//          for(auto each_key : vCovKFs)
//          {
//            cv::imshow("each_key pose",each_key->GetPose());
//            //cout<<mPosevector<<endl;
//            mPosevector=(each_key->GetPose())*mPosevector;
//            float x3D=mPosevector.at<float>(0,0);
//            float y3D=mPosevector.at<float>(1,0);
//            float x2D=x3D*cos(M_PI/4)+500;
//            float y2D=y3D*cos(M_PI/4)+500;
//            cv::circle(mMap2D, cv::Point2f(x2D,y2D),0,255);
//            cv::imshow("map2d",mMap2D);
//
//            std::cout<<mPosevector.at<float>(0,0)<<"," <<endl;
//            std::cout<<mPosevector.at<float>(1,0)<<","  <<endl;
//            std::cout<<mPosevector.at<float>(2,0)<<","  <<endl;
//            std::cout<<mPosevector.at<float>(3,0)<<"\n"<<endl;
//          }
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    cv::Mat CW2 =(*vit)->GetStereoCenter();
//                  float x3D=CW2.at<float>(0,0);
//                  float y3D=CW2.at<float>(1,0);
//                  float x2D=x3D*10+750;
//                  float y2D=y3D*10+750;
//
//                  cv::circle(mMap2D, cv::Point2f(x2D,y2D),0,255);
//                  cv::imshow("map2d",mMap2D);
//                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
//                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
      cout<<"CmaeraPose Matrix :\n"<<mCameraPose<<endl;
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;

      cout<<"OpenGL Matrix ：\n"<<M<<endl;
    }
    else
        M.SetIdentity();
}

void MapDrawer::InitMat(cv::Mat& m,float* num)
{
  for(int i=0;i<m.rows;i++)
    for(int j=0;j<m.cols;j++)
      m.at<float>(i,j)=*(num+i*m.rows+j);
}


void MapDrawer::Draw2DMap() {


}



} //namespace ORB_SLAM
