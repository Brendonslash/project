
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

#include <math.h>



#define  DEGTORAD 0.0174532925199432957f


struct point{
  float x;
  float y;

  point(float a,float b){
    x=a;
    y=b;
  }

  float distance(point p){
    return sqrt(pow(x-p.x,2) + pow(y-p.y,2));
  }

  float angle(point p){
    return atan((p.y-y)/(p.x-x));
  }
  float cenx(point p){
    return (x+p.x)/2;
  }
  
  float ceny(point p){
    return (y+p.y)/2;
  }  
};


namespace cs296
{
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
    b2Body* longarm;


    float angvel = -5;

    // wheels
      
    {      
    
      b2CircleShape circle;
      circle.m_radius=10.0;
      b2FixtureDef ballfd;
      ballfd.shape=&circle;
      ballfd.density = 0.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.5f;
      ballfd.filter.groupIndex = -1;
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

    {

      b2CircleShape circle;
      circle.m_radius=10.0;
      b2BodyDef bd;
      bd.position.Set(-22.0f, 10.0f);
      b2Body* body1 = m_world->CreateBody(&bd);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      
      anchor.Set(-22.0f, 10.0f);
      jd.Initialize(wheel1, body1, anchor);
      jd.enableMotor = true;
      jd.motorSpeed = 12000.0;
      m_world->CreateJoint(&jd);


      bd.position.Set(0.0f, 10.0f);
      b2Body* body2 = m_world->CreateBody(&bd);

      anchor.Set(0.0f, 10.0f);
      jd.Initialize(wheel2, body2, anchor);
      jd.enableMotor = true;
      jd.motorSpeed = 12000.0;
      m_world->CreateJoint(&jd); 

      bd.position.Set(22.0f, 10.0f);
      b2Body* body3 = m_world->CreateBody(&bd);

      anchor.Set(22.0f, 10.0f);
      jd.Initialize(wheel3, body3, anchor);
      jd.enableMotor = true;
      jd.motorSpeed = 12000.0;
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
      fd->filter.groupIndex = -1;
      WheelConnection->CreateFixture(fd);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      
      anchor.Set(-22.0f, 7.0f);
      jd.Initialize(wheel1, WheelConnection, anchor);
      m_world->CreateJoint(&jd);

      anchor.Set(0.0f, 7.0f);
      jd.Initialize(wheel2, WheelConnection, anchor);
      m_world->CreateJoint(&jd);

      anchor.Set(22.0f, 7.0f);
      jd.Initialize(wheel3, WheelConnection, anchor);
      m_world->CreateJoint(&jd);

    }

    //flycrank

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
      fd->filter.groupIndex = -1;
      Flycrank->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      
      anchor.Set(0.0f, 7.0f);
      jd.Initialize(Flycrank, WheelConnection, anchor);
      m_world->CreateJoint(&jd);

      b2RevoluteJointDef jd1;
      b2Vec2 anchor1;

      anchor1.Set(2*sqrt(2),7.0+2*sqrt(2));
      jd1.Initialize(Flycrank, wheel2, anchor1);
      m_world->CreateJoint(&jd1);
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
      Flycrankrod->SetTransform(b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2) * DEGTORAD);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.groupIndex = -1;
      Flycrankrod->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      
      anchor.Set(2*sqrt(2), 7.0+2*sqrt(2));
      jd.Initialize(Flycrankrod, Flycrank, anchor);
      m_world->CreateJoint(&jd);

    }

    //SupportFlycrank
    {

      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(0,0);

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 1.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      fd1->filter.groupIndex = -1;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.5,b2Vec2(21.5f,12.5f), 90 * DEGTORAD);
      fd1->shape = &bs1;

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->filter.groupIndex = -1;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(3,0.5,b2Vec2(21.5f,17.5f), 90 * DEGTORAD);
      fd2->shape = &bs2;

      SupportFlycrank = m_world->CreateBody(bd);
      SupportFlycrank->CreateFixture(fd1);
      SupportFlycrank->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      
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
      fd->filter.groupIndex = -1;
      staticsupport->CreateFixture(fd);
      

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      
      anchor.Set(21.5f, 20.5);
      jd.Initialize(staticsupport, SupportFlycrank, anchor);
      m_world->CreateJoint(&jd);
    }

    //staticbars

    {
      b2PolygonShape shape;
      shape.SetAsBox(10.0f, 0.25f);
  
      b2BodyDef bd;
      bd.type = b2_staticBody;
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.groupIndex = -1;

      bd.position.Set(36.0f, 8.0f);
      staticbardown = m_world->CreateBody(&bd);
      staticbardown->CreateFixture(fd);

      bd.position.Set(36.0f, 12.0f);
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
      fd->filter.groupIndex = 1;

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(3.0f,0.25f,b2Vec2(32.0f,12.0f), 0);
      fd->shape = &bs1;
      lowerpiston->CreateFixture(fd);


      fd->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(1.5,1.75,b2Vec2(32.0f,10.0f), 0);
      fd->shape = &bs2;
      lowerpiston->CreateFixture(fd);

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(3.0f,0.25f,b2Vec2(32.0f,8.0f), 0);
      fd->shape = &bs3;
      lowerpiston->CreateFixture(fd);
      /*
      fd->shape = new b2PolygonShape;
      b2PolygonShape bs4;
      bs4.SetAsBox(10.0f,0.25f,b2Vec2(42.0f,10.0f), 0);
      fd->shape = &bs4;
      lowerpiston->CreateFixture(fd);
      */

    }

    //prismatic joint
/*
    {
      b2PrismaticJointDef prismaticJointDef;
      b2Vec2 worldAxis(1,0);
      b2Vec2 anchor(32.0f,12.0f);
      prismaticJointDef.Initialize(lowerpiston,staticbarup,anchor,worldAxis);
      prismaticJointDef.lowerTranslation = 3.0f;
      prismaticJointDef.upperTranslation = 3.0f;
      prismaticJointDef.enableLimit = true;
      prismaticJointDef.maxMotorForce   = 1.0;
      prismaticJointDef.motorSpeed = 0.0;
      prismaticJointDef.enableMotor = true;

      m_world->CreateJoint(&prismaticJointDef);
    }
*/
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
      fd->filter.groupIndex = 1;

      fd->shape = new b2PolygonShape;
      b2PolygonShape bs;
      bs.SetAsBox(p1.distance(p2)/2,0.5f,b2Vec2(p1.cenx(p2),p1.ceny(p2)), p1.angle(p2));
      fd->shape = &bs;
      longarm->CreateFixture(fd);


      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      
      anchor.Set(0.0f, 7.0);
      jd.Initialize(longarm, WheelConnection, anchor);
      m_world->CreateJoint(&jd);

      anchor.Set(30.5f,10.0f);
      jd.Initialize(longarm, lowerpiston, anchor);
      m_world->CreateJoint(&jd);   
       

   } 


  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
