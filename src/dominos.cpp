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
    b2RevoluteJointDef j;
    b2Joint* desjoin;
    b2Body* Box;

    b2Body* help1;
    b2Body* help2;
    

/*
    void dominos_t :: keyboard(unsigned char key)
    {
        switch(key){

          case('q'):
            point p1(help1->GetPosition());
            point p2(help2->GetPosition());
            float theta1 = atan((22.0-p1.y)/(21.5-p1.x));
            float theta2 =  atan((p2.y-p1.y)/p2.x-p1.x);
            float theta = theta1 - theta2 + (3.14)/2;

            point p( p1.x - (p1.distance(p2)/2 * sin(theta1+ (3.14/2))) ,p1.y - (p1.distance(p2)/2 * cos(theta1 + (3.14/2))));
            
            cout<<theta2/DEGTORAD<<theta1/DEGTORAD<<endl;
            cout<<p1.x<<p1.y<<endl;
            cout<<p2.x<<p2.y<<endl;

            armrest->SetTransform( armrest->GetPosition(), -1 );


            Radiusbar->SetTransform(b2Vec2(p.x,p.y),theta);
            m_world->DestroyJoint(desjoin);
            wheel1->SetAngularVelocity(5);
            wheel2->SetAngularVelocity(5);
            wheel3->SetAngularVelocity(5);
            j.Initialize(Radiusbar,SupportFlycrank,b2Vec2(21.5,22.0));
            desjoin = m_world->CreateJoint(&j);
            break;
          
          
        }

    }*/
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
    b2Body* b1;  
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

    float angvel = 0;

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

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0;
      fd->friction = 0.5;
      fd->restitution = 0.f;
      fd->filter.maskBits=0x0000;
     // fd->filter.groupIndex = -1;

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs;
      bs.SetAsBox(2,0.5,b2Vec2(21.5f,12.5f), 90 * DEGTORAD);
      fd->shape = &bs;
      SupportFlycrank->CreateFixture(fd);


      fd->shape = new b2PolygonShape;
      //fd->filter.groupIndex = 1;
      bs.SetAsBox(5,0.5,b2Vec2(21.5f,19.5f), 90 * DEGTORAD);
      fd->shape = &bs;
      SupportFlycrank->CreateFixture(fd);


      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      jd.collideConnected = false;
      anchor.Set(21.5, 10.5);
      jd.Initialize(Flycrankrod, SupportFlycrank, anchor);
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
      fd->density = 1.0;
      fd->friction = 0.5;
      fd->restitution = 0.f;
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
      lowerpiston->CreateFixture(fd);

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs6;
      bs6.SetAsBox(1.0f,1.0f,b2Vec2(32.0f,6.75f),0);
      fd->shape = &bs6;
      lowerpiston->CreateFixture(fd);


    //  lowerpiston->SetLinearVelocity( b2Vec2(-10,0));

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

      shape.SetAsBox(0.01,0.01);
      help1 = m_world->CreateBody(&bd);
      help1->SetTransform(b2Vec2(39.0,21.0),0);

      shape.SetAsBox(0.01,0.01);
      help2 = m_world->CreateBody(&bd);
      help2->SetTransform(b2Vec2(19.0,16.0),0);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      jd.collideConnected = false;
      anchor.Set(39.0,21.0);

      jd.Initialize(Combinationlever, Radiusbar, anchor);
      m_world->CreateJoint(&jd);

      anchor.Set(21.5,135/8);
      jd.Initialize(SupportFlycrank, Radiusbar, anchor);
      desjoin = m_world->CreateJoint(&jd);

      anchor.Set(39.0,21.0);
      jd.Initialize(help1, Radiusbar, anchor);
      desjoin = m_world->CreateJoint(&jd);

      anchor.Set(19.0,16.0);
      jd.Initialize(help2, Radiusbar, anchor);
      desjoin = m_world->CreateJoint(&jd);


   }

   //Liftinglink
   {
     b2PolygonShape shape;

      point p2(17.0,24.0);
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
      point p1(17.0,24.0);
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
      anchor.Set(17.0,24.0);
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
      fd->density = 1.0;
      fd->friction = 0.5;
      fd->restitution = 0.f;
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
      bs.SetAsBox(2.0f,2.0f,b2Vec2(48.8,19.0f), 0);
      fd->shape = &bs;
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
      fd->filter.maskBits = 0x0000;

      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(47.8-1.5,16.75);
      vertices[1].Set(47.8-2.5,13.25);
      vertices[2].Set(47.8+1.5,16.75);
      vertices[3].Set(47.8+2.5,13.25);
      poly.Set(vertices, 4);

      
      bd.position.Set(0, 0);
      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);

      vertices[0].Set(47.8-2.5,16.75-0.1);
      vertices[1].Set(47.8-3.5,13.25);
      vertices[2].Set(47.8-3.5,16.75-0.1);
      vertices[3].Set(47.8-3.5,16.75-0.1);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);

      vertices[0].Set(47.8+2.5,16.75-0.1);
      vertices[1].Set(47.8+3.5,13.25);
      vertices[2].Set(47.8+3.5,16.75-0.1);
      vertices[3].Set(47.8+3.5,16.75-0.1);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);


      vertices[0].Set(47.8-3.5,6.75);
      vertices[1].Set(47.8-3.5,16.75);
      vertices[2].Set(47.8-4.5,16.75);
      vertices[3].Set(47.8-4.5,6.75);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);

      vertices[0].Set(47.8+3.5,6.75);
      vertices[1].Set(47.8+3.5,16.75);
      vertices[2].Set(47.8+4.5,16.75);
      vertices[3].Set(47.8+4.5,6.75);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);

      vertices[0].Set(47.8+4.5,6.75);
      vertices[1].Set(47.8+4.5,21.25);
      vertices[2].Set(47.8+5.5,21.25);
      vertices[3].Set(47.8+5.5,6.75);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);

      vertices[0].Set(47.8-4.5,6.75);
      vertices[1].Set(47.8-4.5,21.25);
      vertices[2].Set(47.8-5.5,21.25);
      vertices[3].Set(47.8-5.5,6.75);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);

      vertices[0].Set(47.8-5.5,6.75);
      vertices[1].Set(47.8+5.5,6.75);
      vertices[2].Set(47.8+5.5,5.75);
      vertices[3].Set(47.8-5.5,5.75);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);

      vertices[0].Set(47.8-5.5,21.25);
      vertices[1].Set(47.8+5.5,21.25);
      vertices[2].Set(47.8+5.5,22.25);
      vertices[3].Set(47.8-5.5,22.25);
      poly.Set(vertices, 4);

      fd->shape = new b2PolygonShape;
      fd->shape = &poly;

      Box = m_world->CreateBody(&bd);
      Box->CreateFixture(fd);


    }

	
    //////I will add steam inlets and outlets afterwards
  }
	
void dominos_t::step(settings_t* settings){
		base_sim_t::step(settings);}
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
