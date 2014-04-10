#include <iostream>
#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"
//#include "main.cpp"

#include <math.h>
#include "callbacks.hpp"

cs296::settings_t settings;



#define  DEGTORAD 0.0174532925199432957f
/*! The Point struct stores the co-ordinates of a point.  */
struct point{
  float x;
  float y;
	/**
	 * The class contains various functions to manipulate and
	 * evaluate properties of a point.
	 */

  point(float a,float b){
    x=a;
    y=b;
  }

  point(b2Vec2 v){
    x=v.x;
    y=v.y;
  }


  float distance(point p){	//!< Distance between two points 
    return sqrt(pow(x-p.x,2) + pow(y-p.y,2));
  }

  float angle(point p){	//!< Angle made by a line
    return atan((p.y-y)/(p.x-x));
  }
  float cenx(point p){ //!< X-cordinate of the midpoint
    return (x+p.x)/2;
  }
  
  float ceny(point p){ //!< Y-cordinate of the midpoint
    return (y+p.y)/2;
  }  
};


namespace cs296
{

    b2Body* b1;  
    b2Body* wheel1;
    b2Body* wheel2;
    b2Body* wheel3;
    b2Body* WheelConnection;
    b2Body* Flycrank;
    b2Body* Flycrankrod;
    b2Body* SupportFlycrank;
    b2Body* staticsupport;
    b2Body* staticbardown;
    b2Body* staticbarup;
    b2Body* lowerpiston;
    b2Body* upperpiston;
    b2Body* longarm;
    b2Body* Unionlink;
    b2Body* Combinationlever;
    b2Body* Radiusbar;
    b2Body* Liftinglink;
    b2Body* armrest;
    b2Body* Liftingarm;
    b2Body* Reversearm;
    b2Body* Renchrod;
    b2Body* pistonrest;
    b2RevoluteJointDef ju;
    b2Joint* desjoin;
    b2Joint* desprism;
    b2Body* Box;
    b2Body* train;
    b2Body* help1;
    b2Body* help2;
    bool b = true;
    float angvel = 5;
    
      b2RevoluteJointDef movj;
      b2PrismaticJointDef pjoint;
      b2WheelJointDef wjoint;

    void dominos_t :: keyboard(unsigned char key)
    {   

        static bool when = true;

        switch(key){


          case('l'):

            point pl((float)help1->GetPosition().x+21.5,(float)help1->GetPosition().y+135.0/8.0);
            point pu((float)help2->GetPosition().x+21.5,(float)help2->GetPosition().y+24.0);

            if(when)         
            {   
            while((upperpiston->GetPosition().x > 0.1 || upperpiston->GetPosition().x < -0.1) ||  (upperpiston->GetLinearVelocity().x > 0)) 
            {
              callbacks_t::keyboard_cb('o',1,1); 
              callbacks_t::display_cb();
            }

            callbacks_t::keyboard_cb('r',1,1);

            callbacks_t::keyboard_cb('o',1,1); 
            for(int i =0;i<60;i++){
              ball[i]->SetTransform(b2Vec2(10000,10000),0);
            }
            
            wheel1->SetAngularVelocity(0);
            wheel2->SetAngularVelocity(0);
            wheel3->SetAngularVelocity(0);

            m_world->DestroyJoint(desjoin);

            b = false;

            for(int j=0;j<200;j++){
             armrest->SetTransform( armrest->GetPosition(), armrest->GetAngle()-0.01 );
            
            
            
            for(int i = 0;i<10000;i++)
            { callbacks_t::keyboard_cb('o',1,1);} 
            callbacks_t::display_cb();
            }

            b = true;

            ju.Initialize(Radiusbar,SupportFlycrank,b2Vec2(pu.x,pu.y));
            desjoin = m_world->CreateJoint(&ju);

            wheel1->SetAngularVelocity(angvel);
            wheel2->SetAngularVelocity(angvel);
            wheel3->SetAngularVelocity(angvel);

            callbacks_t::keyboard_cb('p',1,1);
            when = !when;
            
            break;
          }
          else
          {
            while((upperpiston->GetPosition().x > 0.1 || upperpiston->GetPosition().x < -0.1) ||  (upperpiston->GetLinearVelocity().x < 0)) 
            {
              callbacks_t::keyboard_cb('o',1,1); 
              callbacks_t::display_cb();
            }

            Radiusbar->SetLinearVelocity(b2Vec2(0,0));
            Combinationlever->SetLinearVelocity(b2Vec2(0,0));

            
            wheel1->SetAngularVelocity(0);
            wheel2->SetAngularVelocity(0);
            wheel3->SetAngularVelocity(0);

            m_world->DestroyJoint(desjoin);
            
            for(int j=0;j<120;j++){
            armrest->SetTransform( armrest->GetPosition(), armrest->GetAngle()+0.01 );
            upperpiston->ApplyForce(b2Vec2(-90,0),b2Vec2(0,0),true);
            for(int i = 0;i<100000;i++)
            { callbacks_t::keyboard_cb('o',1,1);} 
            callbacks_t::display_cb();
            }
            
            ju.Initialize(Radiusbar,SupportFlycrank,b2Vec2(help1->GetPosition().x + 21.5 ,help1->GetPosition().y + 135/8));
            desjoin = m_world->CreateJoint(&ju);

            callbacks_t::keyboard_cb('r',1,1);
            
            wheel1->SetAngularVelocity(-1*angvel);
            wheel2->SetAngularVelocity(-1*angvel);
            wheel3->SetAngularVelocity(-1*angvel);

            callbacks_t::keyboard_cb('p',1,1);

            when = !when;

            break;
          }


        }

    }
  /**  
   * The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  
  dominos_t::dominos_t()
  {
    //Ground
    /*!  
     * \section sec Blocks of objects
     * \subsection a17 common datatypes
     * There are some common data types used repeatedly.
     * There are data types used in defining the shape of object more precisely entity such as
     * \li \b "b2EdgeShape" for shapes resembling edges or line segments.This class has Set(float,float) 
     * as a member function,it takes the starting and the ending positions as inputs giving the line joining them as output.
     * \li \b "b2PolygonShape" for defining Polygons.This class contains SetasBox(float,float) as a member function to set the width 
     * and height of the box if the shape is a rectangle.
     * \li \b "b2CircleShape" for defining Circles,so it obviously will have radius(float),center(vec2(float,float)) variables in it.
     * 
     * \b "b2RevoluteJointDef" is datatype for defining revolute joints. It is created using
     * a member function Initialise(bodyA, bodyB, anchor).
     * 
     * \b m_world defines the world in which the objects interact.It basically is the universe in which the entities are present.
     * There can be multiple worlds.
     * 
     * \b "b2BodyDef" is the data type used to define the body.
     * 
     * \b "b2FixtureDef" defines the fixture of the body.Fixture is defined for a particular Body.It contains the properties of the 
     * body such as mass,restitution,state etc,mostly everything except the dimensions of the body(that is shape).
     * 
     * \b "CreateFixture" helps Create a fixture which is defined with some particular propreties.
     * 
     */ 
    
    {
      /*!
       * \subsection a16 Ground
       * \li \b "b1" is the pointer to the class which contains the properties of Ground like its fixture,shape and state.
       * \li \b "shape" defines its shape,here it is a line segment so b2EdgeShape is used.
       * \li \b "bd" variable sets the body of Ground. 
       */
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }


    b2RevoluteJointDef cj;
    b2Vec2 anc;

    

    // wheels
      
    {      
      /**
       * Here we define the wheels of our steam engine.
       * The maskBits when set to 0 make an object immune to
       * collisions. This is done because we just want the wheels to rotate
       * because of the shaft motion and not collide with them.
       * It has been made of type kinematic to test it when there is no steam
       * in which case a constant angular velocity has been applied to it so that
       * we can test the correctness of the system.
       */
      b2CircleShape circle;
      circle.m_radius=10.0;
      b2FixtureDef ballfd;
      ballfd.shape=&circle;
      ballfd.density = 10.0f;
      ballfd.friction = 0.5f;
      ballfd.restitution = 0.5f;
      ballfd.filter.maskBits= 0x0000;
      b2BodyDef ballbd;
      ballbd.type=b2_dynamicBody;


      ballbd.position.Set(-22.0f,10.0f);
      wheel1=m_world->CreateBody(&ballbd);
      wheel1->CreateFixture(&ballfd);
      wheel1->SetAngularVelocity(angvel);


      ballbd.position.Set(-0.0f,10.0f);
      wheel2=m_world->CreateBody(&ballbd);
      wheel2->CreateFixture(&ballfd);
      wheel2->SetAngularVelocity(angvel);


      ballbd.position.Set(22.0f,10.0f);
      wheel3=m_world->CreateBody(&ballbd);
      wheel3->CreateFixture(&ballfd);
      wheel3->SetAngularVelocity(angvel);

      b=true;

    }

    //revolute joint at centers of wheels

    
    
      /**
       * Revolute joints have been set at the center of the wheel
       * to allow it to rotate and connect it to the. //Check
       */
    {
      b2CircleShape circle;
      circle.m_radius=10.0;
      b2BodyDef bd;
      bd.position.Set(-22.0f, 10.0f);
      b2Body* body1 = m_world->CreateBody(&bd);

      b2RevoluteJointDef jd;
      jd.collideConnected = false;

      jd.bodyA = wheel1;
      jd.bodyB = body1;
      jd.collideConnected = false;
      jd.localAnchorA.Set(0,0);
      jd.localAnchorB.Set(0,0);

      m_world->CreateJoint(&jd);


      bd.position.Set(0.0f, 10.0f);
      b2Body* body2 = m_world->CreateBody(&bd);

      
      jd.bodyA = wheel2;
      jd.bodyB = body2;
      jd.collideConnected = false;
      jd.localAnchorA.Set(0,0);
      jd.localAnchorB.Set(0,0);

      m_world->CreateJoint(&jd); 

      bd.position.Set(22.0f, 10.0f);
      b2Body* body3 = m_world->CreateBody(&bd);

      jd.bodyA = wheel3;
      jd.bodyB = body3;
      jd.collideConnected = false;
      jd.localAnchorA.Set(0,0);
      jd.localAnchorB.Set(0,0);
      m_world->CreateJoint(&jd); 

    }

    //wheel connection

    {
      b2PolygonShape shape;
      shape.SetAsBox(22.0f, 0.5f);
  
      b2BodyDef bd;
      bd.position.Set(0.0f, 7.0f);
      bd.type = b2_dynamicBody;
      WheelConnection = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits= 0x0000;
      WheelConnection->CreateFixture(fd);

      b2RevoluteJointDef jd;
      jd.collideConnected = false;
	
      jd.bodyA = wheel1;
      jd.bodyB = WheelConnection;
      jd.collideConnected = false;
      jd.localAnchorA.Set(0,-3.0);
      jd.localAnchorB.Set(-22.0,0);
	m_world->CreateJoint(&jd);

	jd.bodyA = wheel2;
      jd.bodyB = WheelConnection;
      jd.collideConnected = false;
      jd.localAnchorA.Set(0,-3.0);
      jd.localAnchorB.Set(0,0);
	m_world->CreateJoint(&jd);

	jd.bodyA = wheel3;
      jd.bodyB = WheelConnection;
      jd.collideConnected = false;
      jd.localAnchorA.Set(0,-3.0);
      jd.localAnchorB.Set(22.0,0);
	m_world->CreateJoint(&jd);      
    }

    //flycrank
	//later to local joint
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.0f, 0.5f);
  
      b2BodyDef bd;
      bd.position.Set(0.0f, 9.0f);
      bd.type = b2_dynamicBody;
      Flycrank = m_world->CreateBody(&bd);
      Flycrank->SetTransform(b2Vec2(sqrt(2),7.0+sqrt(2)), 45 * DEGTORAD);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits= 0x0000;
      Flycrank->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      /*b2Vec2 anchor;
      
      anchor.Set(0.0f, 7.0f);

      jd.Initialize(Flycrank, WheelConnection, anchor);
      m_world->CreateJoint(&jd);

      anchor.Set(2*sqrt(2),7.0+2*sqrt(2));
      jd.Initialize(Flycrank, wheel2, anchor);
      m_world->CreateJoint(&jd);*/

	jd.bodyA = Flycrank;
      jd.bodyB = WheelConnection;
      jd.collideConnected = false;
      jd.localAnchorA.Set(-2.0,0.0);
      jd.localAnchorB.Set(0,0);
	m_world->CreateJoint(&jd);

	jd.bodyA = Flycrank;
      jd.bodyB = wheel2;
      jd.collideConnected = false;
      jd.localAnchorA.Set(2.0,0);
      jd.localAnchorB.Set(2*sqrt(2),-3.0+2*sqrt(2));
	m_world->CreateJoint(&jd);
    }

    //flycrankrod

    {
      b2PolygonShape shape;

      point p1(2*sqrt(2),7+2*sqrt(2));
      point p2(21.5,10.5);

      shape.SetAsBox( p1.distance(p2)/2 ,0.5) ;
  
      b2BodyDef bd;
      bd.position.Set(0.0f, 9.0f);
      bd.type = b2_dynamicBody;
      Flycrankrod = m_world->CreateBody(&bd);
      Flycrankrod->SetTransform(b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2));
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits= 0x0000;
      Flycrankrod->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      
      anchor.Set(2*sqrt(2), 7.0+2*sqrt(2));
      jd.Initialize(Flycrankrod, Flycrank, anchor);
      m_world->CreateJoint(&jd);
	/*
	jd.bodyA = Flycrank;
      jd.bodyB = Flycrankrod;
      jd.collideConnected = false;

      jd.localAnchorA.Set(2.0,0);
      jd.localAnchorB.Set(-p1.distance(p2)/2,0);
	m_world->CreateJoint(&jd);

  jd.bodyA = wheel2;
      jd.bodyB = Flycrankrod;
      jd.collideConnected = false;

      jd.localAnchorA.Set(2.0,0);
      jd.localAnchorB.Set(-p1.distance(p2)/2,0);
  m_world->CreateJoint(&jd);
  */
    }

    //SupportFlycrank
    {

      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(0,0);
      SupportFlycrank = m_world->CreateBody(bd);
      help1 = m_world->CreateBody(bd);
      help2 = m_world->CreateBody(bd);

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0;
      fd->friction = 0.5;
      fd->restitution = 0.f;
      fd->filter.maskBits=0x0000;

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs;
      bs.SetAsBox(2,0.5,b2Vec2(21.5f,12.5f), 90 * DEGTORAD);
      fd->shape = &bs;
      SupportFlycrank->CreateFixture(fd);


      fd->shape = new b2PolygonShape;
      bs.SetAsBox(5,0.5,b2Vec2(21.5f,19.5f), 90 * DEGTORAD);
      fd->shape = &bs;
      SupportFlycrank->CreateFixture(fd);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      jd.collideConnected = false;

      anchor.Set(21.5, 10.5);
      jd.Initialize(Flycrankrod, SupportFlycrank, anchor);
      m_world->CreateJoint(&jd); 

     
      bd->position.Set(21.5,138/8);
      fd->shape = new b2PolygonShape;
      bs.SetAsBox(1,1,b2Vec2(21.5,135/8), 0);
      fd->shape = &bs;
     // help1->CreateFixture(fd);

      anchor.Set(21.5,135/8);
      jd.Initialize(help1, SupportFlycrank, anchor);
      m_world->CreateJoint(&jd); 
      
      bd->position.Set(21.5,24.0);
      fd->shape = new b2PolygonShape;
      bs.SetAsBox(1,1,b2Vec2(21.5,24.0), 0);
      fd->shape = &bs;
     // help2->CreateFixture(fd);
      
      
      anchor.Set(21.5,24.0);
      jd.Initialize(help2, SupportFlycrank, anchor);
       m_world->CreateJoint(&jd);

    }

    //staticsupport

    {
      b2PolygonShape shape;
      shape.SetAsBox(3.0f, 1.0f);
  
      b2BodyDef bd;
      bd.position.Set(18.5f, 21.5f);
      bd.type = b2_staticBody;
      staticsupport = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits= 0x0000;

      staticsupport->CreateFixture(fd);
      
      b2RevoluteJointDef jd;
      jd.collideConnected = false;
      b2Vec2 anchor;
      
      anchor.Set(21.5f, 20.5);
      jd.Initialize(staticsupport, SupportFlycrank, anchor);
      m_world->CreateJoint(&jd);
    }

    //staticbars

    {
      b2PolygonShape shape;
      shape.SetAsBox(8.0f, 0.25f);
  
      b2BodyDef bd;
      bd.type = b2_staticBody;
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits = 0x0000;

      bd.position.Set(34.0f, 8.0f);
      staticbardown = m_world->CreateBody(&bd);
      staticbardown->CreateFixture(fd);

      bd.position.Set(34.0f, 12.0f);
      staticbarup = m_world->CreateBody(&bd);
      staticbarup->CreateFixture(fd);



    }

    //lowerpiston
    

    {

      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(0,0);
      lowerpiston = m_world->CreateBody(bd);

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 6;//change
      fd->friction = 0;
      fd->restitution = 1.f;
      fd->filter.maskBits = 0x0000;

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(3.0f,0.25f,b2Vec2(32.0f,12.0f), 0);
      fd->shape = &bs1;
      lowerpiston->CreateFixture(fd);


      fd->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(1.5,1.7,b2Vec2(32.0f,10.0f), 0);
      fd->shape = &bs2;
      lowerpiston->CreateFixture(fd);

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(3.0f,0.25f,b2Vec2(32.0f,8.0f), 0);
      fd->shape = &bs3;
      lowerpiston->CreateFixture(fd);
      
      fd->shape = new b2PolygonShape;
      b2PolygonShape bs4;
      bs4.SetAsBox(7.0f,0.25f,b2Vec2(40.5f,10.0f), 0);
      fd->filter.maskBits = 0x0000;
      fd->shape = &bs4;
      lowerpiston->CreateFixture(fd);
      
//////////////////////////////////////////////////////////////////////////////////////////// take care

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs5;
      bs5.SetAsBox(0.25f,3.0f,b2Vec2(47.75f,10.0f),0);
      fd->shape = &bs5;
      fd->filter.categoryBits=0x0005;
      fd->filter.maskBits=0x0006|0x0003;
      lowerpiston->CreateFixture(fd);

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs6;
      bs6.SetAsBox(1.0f,1.0f,b2Vec2(32.0f,6.75f),0);
      fd->shape = &bs6;
      lowerpiston->CreateFixture(fd);


    //  lowerpiston->SetLinearVelocity( b2Vec2(-100,0));

    }

    //prismatic joint
//check------------------------------------------------------------------
   {
      b2PrismaticJointDef prismaticJointDef;
      b2Vec2 worldAxis(1,0);
	//cout << lowerpiston->GetWorldCenter().x << endl;
	//cout << lowerpiston->GetWorldCenter().y << endl;//debugggggg
      b2Vec2 anchor(32.0f,12.0f);
	//b2Vec2 anc(0,0);
      prismaticJointDef.Initialize(lowerpiston,staticbarup,anc,worldAxis);
      prismaticJointDef.collideConnected = false;
      prismaticJointDef.lowerTranslation = 3.0f;
      prismaticJointDef.upperTranslation = 3.0f;
      prismaticJointDef.enableLimit = false;
      prismaticJointDef.maxMotorForce   = 1.0;
      prismaticJointDef.motorSpeed = 0.0;
      prismaticJointDef.enableMotor = false;

      m_world->CreateJoint(&prismaticJointDef);
}

//-----------------------------------Check

    //longarm ---- connection from lower piston to wheel2

    {
      point p1(0.0f,7.0f);
      point p2(30.5f,10.0f);

      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;  
      longarm = m_world->CreateBody(bd);

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0;
      fd->friction = 0.0;
      fd->restitution = 1.0f;
      fd->filter.maskBits= 0x0000;

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs;
      bs.SetAsBox(p1.distance(p2)/2,0.5f,b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2));
      fd->shape = &bs;
      longarm->CreateFixture(fd);


      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      jd.collideConnected = false;
      anchor.Set(0.0f, 7.0);
      jd.Initialize(longarm, WheelConnection, anchor);
      m_world->CreateJoint(&jd);


      anchor.Set(30.5f,10.0f);
	//cout << lowerpiston->GetWorldCenter().x << endl;
	//cout << lowerpiston->GetWorldCenter().y << endl;
	/*jd.bodyA = longarm;
      jd.bodyB = lowerpiston;
      jd.collideConnected = false;
      jd.localAnchorA.Set(p1.distance(p2)/2,0.0);
      jd.localAnchorB.Set(22.0,0);
	m_world->CreateJoint(&jd);*/  
      jd.Initialize(longarm, lowerpiston, anchor);
      m_world->CreateJoint(&jd);   

   } 

   //armrest
   {
      
      b2CircleShape circle;
      circle.m_radius=1.0;
      b2FixtureDef ballfd;
      ballfd.shape=&circle;
      ballfd.density = 10.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.5f;
      ballfd.filter.maskBits= 0x0000;
      b2BodyDef ballbd;
      ballbd.type=b2_kinematicBody;


      ballbd.position.Set(24.0f,28.0f);
      armrest=m_world->CreateBody(&ballbd);
      armrest->CreateFixture(&ballfd);

   }

   //Unionlink
   {
      b2PolygonShape shape;

      point p1(32.0,5.75);
      point p2(38.0,7.0);

      shape.SetAsBox( p1.distance(p2)/2 ,0.25) ;
  
      b2BodyDef bd;
      bd.position.Set(0.0f, 9.0f);
      bd.type = b2_dynamicBody;
      Unionlink = m_world->CreateBody(&bd);
      Unionlink->SetTransform(b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2));
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      //fd->filter.groupIndex = -1;
      fd->filter.maskBits= 0x0000;
      Unionlink->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      jd.collideConnected = false;
      anchor.Set(32.0,5.75);
      jd.Initialize(Unionlink, lowerpiston, anchor);
      m_world->CreateJoint(&jd);

   }

   //Combinationlever
   {
      b2PolygonShape shape;

      point p2(39.0,21.0);
      point p1(38.0,7.0);

      shape.SetAsBox( p1.distance(p2)/2 ,0.25) ;
  
      b2BodyDef bd;
      bd.position.Set(0.0f, 9.0f);
      bd.type = b2_dynamicBody;
      Combinationlever = m_world->CreateBody(&bd);
      Combinationlever->SetTransform(b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2));
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits= 0x0000;
      Combinationlever->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      jd.collideConnected = false;
      anchor.Set(38.0,7.0);
      jd.Initialize(Unionlink, Combinationlever, anchor);
      m_world->CreateJoint(&jd);

   }

   //Radiusbar
   {
      b2PolygonShape shape;

      point p1(39.0,21.0);
      point p2(19.0,16.0);

      shape.SetAsBox( p1.distance(p2)/2 ,0.25) ;
  
      b2BodyDef bd;
      bd.position.Set(0.0f, 9.0f);
      bd.type = b2_dynamicBody;
      Radiusbar = m_world->CreateBody(&bd);
      Radiusbar->SetTransform(b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2));
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits= 0x0000;
      Radiusbar->CreateFixture(fd);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;

      
      jd.collideConnected = false;
      anchor.Set(39.0,21.0);

      jd.Initialize(Combinationlever, Radiusbar, anchor);
      m_world->CreateJoint(&jd);

      anchor.Set(21.5,135/8);
      movj.Initialize(SupportFlycrank, Radiusbar, anchor);
      desjoin = m_world->CreateJoint(&movj);

      
      

   }

   //Liftinglink
   {
     b2PolygonShape shape;

      point p2(15.0,24.0);
      point p1(19.0,16.0);

      shape.SetAsBox( p1.distance(p2)/2 ,0.25) ;
  
      b2BodyDef bd;
      bd.position.Set(0.0f, 9.0f);
      bd.type = b2_dynamicBody;
      Liftinglink = m_world->CreateBody(&bd);
      Liftinglink->SetTransform(b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2));
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits= 0x0000;
      Liftinglink->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      jd.collideConnected = false;
      anchor.Set(19.0,16.0);
      jd.Initialize(Liftinglink, Radiusbar, anchor);
      m_world->CreateJoint(&jd);

      

   }

   //Liftingarm

   
   {
      point p1(15.0,24.0);
      point p2(24.0f, 28.0f);

      b2PolygonShape shape;
      shape.SetAsBox( p1.distance(p2)/2 ,0.25) ;
  
      b2BodyDef bd;
      bd.position.Set(0.0f, 9.0f);
      bd.type = b2_dynamicBody;
      Liftingarm = m_world->CreateBody(&bd);
      Liftingarm->SetTransform(b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2));
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits= 0x0000;
      Liftingarm->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      jd.collideConnected = false;
      anchor.Set(15.0,24.0);
      jd.Initialize(Liftinglink, Liftingarm, anchor);
      m_world->CreateJoint(&jd);

      b2WeldJointDef* weldJointDef = new b2WeldJointDef();
      weldJointDef->Initialize(Liftingarm, armrest, b2Vec2(24.0,28.0));
      m_world->CreateJoint(weldJointDef);

      anchor.Set(24.0f, 28.0f);
      jd.Initialize(armrest, Liftingarm, anchor);
      m_world->CreateJoint(&jd);

     


   }

   //Reversearm
   {
      point p2(24.0,28.0);
      point p1(28.0f, 24.0f);

      b2PolygonShape shape;
      shape.SetAsBox( p1.distance(p2)/2 ,0.25) ;
  
      b2BodyDef bd;
      bd.position.Set(0.0f, 9.0f);
      bd.type = b2_dynamicBody;
      Reversearm = m_world->CreateBody(&bd);
      Reversearm->SetTransform(b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2));
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits= 0x0000;
      Reversearm->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      jd.collideConnected = false;
      anchor.Set(24.0f, 28.0f);
      jd.Initialize(armrest, Reversearm, anchor);
      m_world->CreateJoint(&jd);

      anchor.Set(25.0f, 27.0f);
      jd.Initialize(armrest, Reversearm, anchor);
      m_world->CreateJoint(&jd);

      
   }

   //Renchrod
   {
      point p2(5.0,24.0);
      point p1(28.0f, 24.0f);

      b2PolygonShape shape;
      shape.SetAsBox( p1.distance(p2)/2 ,0.25) ;
  
      b2BodyDef bd;
      bd.position.Set(0.0f, 9.0f);
      bd.type = b2_kinematicBody;
      Renchrod = m_world->CreateBody(&bd);
      Renchrod->SetTransform(b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2));
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.maskBits= 0x0000;
      Renchrod->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      jd.collideConnected = false;
      anchor.Set(28.0f, 28.0f);
      jd.Initialize(armrest, Renchrod, anchor);
      m_world->CreateJoint(&jd);
   }

   //upperpiston
   {

      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(0,0);
      upperpiston = m_world->CreateBody(bd);

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 10.0;
      fd->friction = 0;
      fd->restitution = 1.f;
      fd->filter.maskBits = 0x0000;
      fd->shape = new b2PolygonShape;
      b2PolygonShape bs;

      bs.SetAsBox(4.0f,0.25f,b2Vec2(42.8,19.0f), 0);
      fd->shape = &bs;
      
      upperpiston->CreateFixture(fd);

///////////////////////////////////////////////////////////////////////////takecare of maskbits when u add particles
      
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
    jd.collideConnected = false;
  //schange reduced by 0.2
      bs.SetAsBox(1.8f,2.25f,b2Vec2(48.6,19.0f), 0);
      fd->shape = &bs;
      fd->filter.categoryBits=0x0005;  ///////////////////////////////////////////
      fd->filter.maskBits=0x0006 | 0x0003;      /////////////////////////////////////////////////
      fd->density=10.f;
      upperpiston->CreateFixture(fd);      

      
      anchor.Set(38.8, 19.0f);
      jd.Initialize(upperpiston, Combinationlever, anchor);
      m_world->CreateJoint(&jd);


    //  upperpiston->ApplyForce( b2Vec2(-1000,0) , b2Vec2(43-(1/7),19),true);

   }

{
	//joint lower
	 b2Vec2 worldAxis(1,0);
	b2Vec2 anc(0,0);
	b2PrismaticJointDef prismaticJointDef2;
	//cout << upperpiston->GetWorldCenter().x << endl;
	//cout << upperpiston->GetWorldCenter().y << endl;//debugggggg
      //b2Vec2 anchor(32.0f,12.0f);
      prismaticJointDef2.Initialize(upperpiston,staticbarup,anc,worldAxis);
      prismaticJointDef2.lowerTranslation = 3.0f;
	    prismaticJointDef2.collideConnected = false;
      prismaticJointDef2.upperTranslation = 3.0f;
      prismaticJointDef2.enableLimit = false;
      prismaticJointDef2.maxMotorForce   = 1.0;
      prismaticJointDef2.motorSpeed = 0.0;
      prismaticJointDef2.enableMotor = false;

      m_world->CreateJoint(&prismaticJointDef2);
    }

//ffffffffffffffffffffffffffffffffffffffffffff
  //Box
    {

      b2PolygonShape shape;
      shape.SetAsBox(4.5f, 0.25f);
  
      b2BodyDef bd;
      bd.type = b2_staticBody;
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->friction = 0.0f;
      fd->restitution = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      //fd->filter.groupIndex = 1;
      fd->filter.maskBits = 0x0006;
      fd->filter.categoryBits=0x0003;

      b2PolygonShape poly;
      b2Vec2 vertices[4];
  //mid
      vertices[0].Set(47.8-2.5,16.75);
      vertices[1].Set(47.8-2.5,13.25);
      vertices[2].Set(47.8+2.5,16.75);
      vertices[3].Set(47.8+2.5,13.25);
      poly.Set(vertices, 4);

      
      bd.position.Set(0, 0);
      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);


      vertices[0].Set(47.8+4.5,6.75);
      vertices[1].Set(47.8+4.5,21.25);
      vertices[2].Set(47.8+5.5,21.25);
      vertices[3].Set(47.8+5.5,6.75);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

    //  Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);

      vertices[0].Set(47.8-4.5,6.75);
      vertices[1].Set(47.8-4.5,21.25);
      vertices[2].Set(47.8-5.5,21.25);
      vertices[3].Set(47.8-5.5,6.75);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

    //  Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);
  // Lower plank
      vertices[0].Set(47.8-5.5,6.75);
      vertices[1].Set(47.8+5.5,6.75);
      vertices[2].Set(47.8+5.5,5.75);
      vertices[3].Set(47.8-5.5,5.75);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

   //   Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);
  //Left upper
      vertices[0].Set(47.8-5.5,21.25); //  Initially it was +-5.5
      vertices[1].Set(47.8-4.5,21.25);
      vertices[2].Set(47.8-4.5,27.25);
      vertices[3].Set(47.8-5.5,27.25);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
      Box->CreateFixture(fd);
      //Right upper
      vertices[0].Set(47.8+4.5,21.25); //  Initially it was +-5.5
      vertices[1].Set(47.8+5.5,21.25);
      vertices[2].Set(47.8+5.5,27.25);
      vertices[3].Set(47.8+4.5,27.25);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
      Box->CreateFixture(fd);
      /*
      vertices[0].Set(47.8-1.5,21.25); //  Initially it was +-5.5
      vertices[1].Set(47.8+1.5,21.25);
      vertices[2].Set(47.8+1.5,22.25);
      vertices[3].Set(47.8-1.5,22.25);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
      Box->CreateFixture(fd); */
   
   //Long Side right
      vertices[0].Set(47.8+2,21.5); //  Initially it was +-5.5
      vertices[1].Set(47.8+3,21.5);
      vertices[2].Set(47.8+2,30.25);
      vertices[3].Set(47.8+3,30.25);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
      Box->CreateFixture(fd);      

      vertices[0].Set(47.8+1.5,25.25); //  Initially it was +-5.5
      vertices[1].Set(47.8+5.5,25.25);
      vertices[2].Set(47.8+5.5,26.25);
      vertices[3].Set(47.8+1.5,26.25);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
     // Box->CreateFixture(fd);
      
      vertices[0].Set(47.8-2.5,30.25); //  Initially it was +-5.5
      vertices[1].Set(47.8+2.5,30.25);
      vertices[2].Set(47.8+2.5,31.25);
      vertices[3].Set(47.8-2.5,31.25);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
      Box->CreateFixture(fd);      

  // Long side left
      vertices[0].Set(47.8-2.8,30.25); //  Initially it was +-5.5
      vertices[1].Set(47.8-2.4,30.25);
      vertices[2].Set(47.8-2.4,21.3);
      vertices[3].Set(47.8-2.8,21.3);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
      Box->CreateFixture(fd);
      
      vertices[0].Set(47.8-0.5,21.3); //  Initially it was +-5.5
      vertices[1].Set(47.8+0.75,21.3);
      vertices[2].Set(47.8+0.75,22.25);
      vertices[3].Set(47.8-0.5,22.25);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
     // Box->CreateFixture(fd);      
      
      vertices[0].Set(47.8+2.5,21.3); //  Initially it was +-5.5
      vertices[1].Set(47.8+4.5,21.3);
      vertices[2].Set(47.8+4.5,22.25);
      vertices[3].Set(47.8+2.5,22.25);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
   //   Box->CreateFixture(fd);            
      
      
    }
  // Steam 
    {
    b2CircleShape shape;
    shape.m_radius=0.22f; //change
    b2BodyDef bd;
  //  bd.position.Set(47.8,27);
    bd.type=b2_dynamicBody;
    bd.bullet=true;
    b2FixtureDef fd;
    fd.shape=&shape;
    fd.restitution=1;
    fd.density=0.05f;
    fd.friction=0.0f;
    fd.filter.categoryBits=0x0006;
    fd.filter.maskBits=0x0003 | 0x0005;
    //0
    for(int i=0;i<30;i++){
      bd.position.Set(47.8-4,10);       ///////////////////////////////////
      ball[i]=m_world->CreateBody(&bd);
      fd1[i]=ball[i]->CreateFixture(&fd);
      
      ball[i]->SetLinearVelocity(b2Vec2(100 + (rand()%10)/10.0,100+(rand()%10)/10.0));
    }
    for(int i=30;i<60;i++){//0
      bd.position.Set(47.8+4,10);       ///////////////////////////////////
      ball[i]=m_world->CreateBody(&bd);
      fd1[i]=ball[i]->CreateFixture(&fd);
      ball[i]->SetLinearVelocity(b2Vec2(-100+(rand()%10)/10.0,100+(rand()%10)/10.0));
    }

  }
  //train
  {
    b2PolygonShape shape;
    b2Vec2 vertices[5];
    vertices[0].Set(-30,17);
    vertices[1].Set(-30,40);
    vertices[2].Set(55,40);
    vertices[3].Set(55,28);
    vertices[4].Set(35,17);
    shape.Set(vertices,5);
    b2BodyDef bd;
    bd.position.Set(0,0);
    bd.type=b2_dynamicBody;
    b2FixtureDef fd;
    fd.shape=&shape;
    fd.filter.maskBits=0x0000;
    train=m_world->CreateBody(&bd);
    train->CreateFixture(&fd);
    
    vertices[0].Set(-23,18);
    vertices[1].Set(-21,18);
    vertices[2].Set(-21,9);
    vertices[3].Set(-23,9);
    vertices[4].Set(-23,9);
    shape.Set(vertices,5);
    fd.shape=&shape;    
    train->CreateFixture(&fd);
    
    vertices[0].Set(-1,18);
    vertices[1].Set(1,18);
    vertices[2].Set(1,9);
    vertices[3].Set(-1,9);
    vertices[4].Set(-1,9);
    shape.Set(vertices,5);
    fd.shape=&shape;    
    train->CreateFixture(&fd);
    
    vertices[0].Set(21,18);
    vertices[1].Set(23,18);
    vertices[2].Set(23,9);
    vertices[3].Set(21,9);
    vertices[4].Set(21,9);
    shape.Set(vertices,5);
    fd.shape=&shape;    
    train->CreateFixture(&fd);
    
    b2RevoluteJointDef rj2;
    rj2.Initialize(wheel1,train,wheel1->GetWorldCenter());
    m_world->CreateJoint(&rj2);
    
    rj2.Initialize(wheel2,train,wheel2->GetWorldCenter());
    m_world->CreateJoint(&rj2);
    
    rj2.Initialize(wheel3,train,wheel3->GetWorldCenter());
    m_world->CreateJoint(&rj2);
    
    rj2.bodyA=wheel1;
    rj2.bodyB=train;
    rj2.collideConnected=false;
    rj2.localAnchorA.Set(0,0);
    rj2.localAnchorB.Set(-22,10);
    m_world->CreateJoint(&rj2);
    
    rj2.bodyA=wheel2;
    rj2.bodyB=train;
    rj2.collideConnected=false;
    rj2.localAnchorA.Set(0,0);
    rj2.localAnchorB.Set(0,10);
    m_world->CreateJoint(&rj2);
    
    rj2.bodyA=wheel3;
    rj2.bodyB=train;
    rj2.collideConnected=false;
    rj2.localAnchorA.Set(0,0);
    rj2.localAnchorB.Set(22,10);
    m_world->CreateJoint(&rj2); 
            
  } 

}
//38.6849  , 32.8396, 47.8
void dominos_t::step(settings_t* settings){
    base_sim_t::step(settings);
      bool fl = (lowerpiston->GetWorldCenter().x + 12.3 < 47.7 ) || (lowerpiston->GetWorldCenter().x + 12.3 > 47.9);
      double magL, magR;
      magL = magR = 200;
      double si = 1;
      if( b )
      {
      if(lowerpiston->GetWorldCenter().x < 35.5) si = 1;
      else si = -1;
      if(lowerpiston->GetWorldCenter().x < 33.2) {
        magL = 200;
        magR = 40;
      }
      else if(lowerpiston->GetWorldCenter().x > 37.8) {
        magL = 40;
        magR = 200;
      }
    //cout<<lowerpiston->GetWorldCenter().x<<endl; 
    for(int i=0;i<60;i++){//change  
      b2Vec2 vel = ball[i]->GetLinearVelocity();    
      double mag = sqrt(vel.x*vel.x + vel.y*vel.y);
      if((ball[i]->GetWorldCenter().y < 21) && fl) {
      
      if(ball[i]->GetWorldCenter().x < lowerpiston->GetWorldCenter().x + 12.3) 
        ball[i]->SetLinearVelocity(b2Vec2(vel.x * magL /mag, vel.y * magL / mag));
      else
        ball[i]->SetLinearVelocity(b2Vec2(vel.x * magR /mag, vel.y * magR / mag));
      }
      else{
       ball[i]->SetLinearVelocity(b2Vec2(vel.x * 100.0 /mag, vel.y * 100.0 / mag));
      }
      if(ball[i]->GetWorldCenter().y>32){
        b2BodyDef bd;
        b2CircleShape shape;
        shape.m_radius=0.22f;
        bd.position.Set(47.8,24);
        
        if(lowerpiston->GetWorldCenter().x < 33.4) bd.position.Set(lowerpiston->GetWorldCenter().x + 12.3 - 0.5,10);
        else if(lowerpiston->GetWorldCenter().x > 37.6) bd.position.Set(lowerpiston->GetWorldCenter().x + 12.3 + 0.5,10);
        
        bd.type=b2_dynamicBody;
        bd.bullet=true;
        b2FixtureDef fd;
        fd.shape=&shape;
        fd.density=5.0f;
        fd.friction=0.0f;
        fd.restitution=1;
        fd.filter.categoryBits=0x0006;
        fd.filter.maskBits=0x0003;  
        ball[i]->DestroyFixture(fd1[i]);
        ball[i]=m_world->CreateBody(&bd);
        fd1[i]=ball[i]->CreateFixture(&fd);
        
        ball[i]->SetLinearVelocity(b2Vec2(si * 100+(rand()%10)/10.0,-100+(rand()%10)/10.0));
      }
    }
  }
  }
    
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
