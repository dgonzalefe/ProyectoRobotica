/*
 *    Copyright (C) 2017 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <math.h>   


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    
    innerModel = new InnerModel("/home/svenminds/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()

{
  try{
      
      RoboCompDifferentialRobot::TBaseState bState;
      differentialrobot_proxy->getBaseState(bState);
      innerModel->updateTransformValues("base",bState.x,0,bState.z,0,bState.alpha,0);
      
      QVec ini;
      float distanceObject;
      const float maxADV=500;
      const float maxROT=0.6;
      
      
      
      if(pick.isActive())
      {
          
           
          
          QVec tr =innerModel->transform("base",pick.getAux(),"world");
          distanceObject=tr.norm2();
        
          
          if(distanceObject > 50)
          {
              
              
              float vROT = atan2(tr.x(),tr.z());;
              float velocidadAvance=distanceObject;
              
              if (vROT > maxROT)
                vROT = maxROT;
              
              if (-vROT < -maxROT) 
                  vROT = -maxROT;
          
              velocidadAvance=maxADV*setGauss(0.3,vROT,0.5)*setSignmoide(distanceObject);
          
          
              differentialrobot_proxy->setSpeedBase(velocidadAvance,vROT);
              
              
            
          }
          else
          {
              
              pick.setActive(false);
              stopRobot();
              //differentialrobot_proxy->setSpeedBase(0,0);
                
          
          
          

          
              
        }
      }
          
    }catch (const Ice::Exception &exc)
  {
    std::cout << exc << std::endl;
  }

}


float SpecificWorker::setGauss(float vADV, float vROT,float h)
{
    
    float lambda = -(pow(vADV, 2.0))/log(h);
    return exp(-pow(vROT, 2.0)/lambda);
    
}

float SpecificWorker::setSignmoide(float distancia)
{
    
    return 1/(1+exp(-distancia))-0.5;
    
}


void SpecificWorker::stopRobot()
{
    
    differentialrobot_proxy->stopBase();

}
void SpecificWorker::setPick(const Pick &mypick)
{
    
    qDebug() << "New target selected: " << mypick.x << mypick.z;
    pick.copy ( mypick.x,mypick.z );
    pick.setActive ( true );
}
  








