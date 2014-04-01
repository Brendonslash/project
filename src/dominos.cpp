/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */
 
 /* Category and masked bits -
  * 0x002 for wheels,
  * 0x003 for rods,
  * 0x001 for base line
  * Wheels should collide with base line, Rods should collide with base line[Just in case]
  */


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

namespace cs296
{	  
  dominos_t::dominos_t()
  {
     
	{
	  b2Body* ground;
	  b2EdgeShape shape;
	  shape.Set(b2Vec2(-200.0f, 0.0f), b2Vec2(200.0f, 0.0f));
	  b2FixtureDef fd;
	  fd.shape=&shape;
	  fd.filter.categoryBits=0x001;
	  fd.filter.maskBits=0xFFFF;
	  b2BodyDef bd;
	  ground = m_world->CreateBody(&bd);
	  ground->CreateFixture(&fd);
	      
     //A sphere and a rectangular block attached to it
	  b2Body* sphere1;
	  b2CircleShape circle;
	  circle.m_radius=20.0;
	  b2FixtureDef ballfd1;
	  ballfd1.shape=&circle;
	  ballfd1.density = 5.0f;
      ballfd1.friction = 1.0f;
      ballfd1.restitution = 0.5f;
      ballfd1.filter.categoryBits=0x002;
      ballfd1.filter.maskBits=0x001|0x002;
      b2BodyDef ballbd1;
      ballbd1.type=b2_dynamicBody;
      ballbd1.position.Set(-25.f,20.f);
      sphere1=m_world->CreateBody(&ballbd1);
      sphere1->CreateFixture(&ballfd1);
    
      b2Body* smallbox1;
	  b2PolygonShape shape1;
	  shape1.SetAsBox(1.f,7.5f);
	  b2FixtureDef fd1;
	  fd1.shape=&shape1;
	  fd1.filter.categoryBits=0x003;
	  fd1.filter.maskBits=0x001;
	  b2BodyDef bd1;
	  bd1.type=b2_dynamicBody;
	  bd1.position.Set(-25.f,12.5f);
	  smallbox1=m_world->CreateBody(&bd1);
	  smallbox1->CreateFixture(&fd1);
	  
	  b2RevoluteJointDef jointDef1;
	  jointDef1.bodyA=sphere1;
	  jointDef1.bodyB=smallbox1;
	  jointDef1.localAnchorA.Set(0,0);
	  jointDef1.localAnchorB.Set(0,7.5);
	 // jointDef.collideConnected=false;
	  //jointdef.Initialize(sphere,smallbox,sphere->GetLocalCenter());
	  m_world->CreateJoint(&jointDef1);
      
     //A sphere and a rectangular block attached to it
	  b2Body* sphere2;
	  b2CircleShape circle2;
	  circle2.m_radius=20.0;
	  b2FixtureDef ballfd2;
	  ballfd2.shape=&circle2;
	  ballfd2.density = 5.0f;
      ballfd2.friction = 1.0f;
      ballfd2.restitution = 0.5f;
      ballfd2.filter.categoryBits=0x002;
      ballfd2.filter.maskBits=0x001;
      b2BodyDef ballbd2;
      ballbd2.type=b2_dynamicBody;
      ballbd2.position.Set(20.f,20.f);
      sphere2=m_world->CreateBody(&ballbd2);
      sphere2->CreateFixture(&ballfd2);
    
      b2Body* smallbox2;
	  b2PolygonShape shape2;
	  shape2.SetAsBox(1.f,7.5f);
	  b2FixtureDef fd2;
	  fd2.shape=&shape2;
	  fd2.filter.categoryBits=0x003;
	  fd2.filter.maskBits=0x001;
	  b2BodyDef bd2;
	  bd2.type=b2_dynamicBody;
	  bd2.position.Set(20.f,12.5f);
	  smallbox2=m_world->CreateBody(&bd2);
	  smallbox2->CreateFixture(&fd2);
	  
	  b2RevoluteJointDef jointDef2;
	  jointDef2.bodyA=sphere2;
	  jointDef2.bodyB=smallbox2;
	  jointDef2.localAnchorA.Set(0,0);
	  jointDef2.localAnchorB.Set(0,7.5);
	 // jointDef.collideConnected=false;
	  //jointdef.Initialize(sphere,smallbox,sphere->GetLocalCenter());
	  m_world->CreateJoint(&jointDef2);
      
   //A sphere and a rectangular block attached to it
	  b2Body* sphere3;
	  b2CircleShape circle3;
	  circle3.m_radius=20.0;
	  b2FixtureDef ballfd3;
	  ballfd3.shape=&circle3;
	  ballfd3.density = 5.0f;
      ballfd3.friction = 1.0f;
      ballfd3.restitution = 0.5f;
      ballfd3.filter.categoryBits=0x002;
      ballfd3.filter.maskBits=0x001;
      b2BodyDef ballbd3;
      ballbd3.type=b2_dynamicBody;
      ballbd3.position.Set(65.f,20.f);
      sphere3=m_world->CreateBody(&ballbd3);
      sphere3->CreateFixture(&ballfd3);
    
      b2Body* smallbox3;
	  b2PolygonShape shape3;
	  shape3.SetAsBox(1.f,7.5f);
	  b2FixtureDef fd3;
	  fd3.shape=&shape3;
	  fd3.filter.categoryBits=0x003;
	  fd3.filter.maskBits=0x001;
	  b2BodyDef bd3;
	  bd3.type=b2_dynamicBody;
	  bd3.position.Set(65.f,12.5f);
	  smallbox3=m_world->CreateBody(&bd3);
	  smallbox3->CreateFixture(&fd3);
	  
	  b2RevoluteJointDef jointDef3;
	  jointDef3.bodyA=sphere3;
	  jointDef3.bodyB=smallbox3;
	  jointDef3.localAnchorA.Set(0,0);
	  jointDef3.localAnchorB.Set(0,7.5);
	 // jointDef.collideConnected=false;
	  //jointdef.Initialize(sphere,smallbox,sphere->GetLocalCenter());
	  m_world->CreateJoint(&jointDef3);
	  
	  b2Body* rod;
	  b2PolygonShape rodshape;
	  rodshape.SetAsBox(45.f,1.f);
	  b2FixtureDef rfd;
	  rfd.shape=&rodshape;
	  rfd.filter.categoryBits=0x003;
	  rfd.filter.maskBits=0x001;
	  b2BodyDef rbd;
	  rbd.type=b2_dynamicBody;
	  rbd.position.Set(20.f,6.f);
	  rod=m_world->CreateBody(&rbd);
	  rod->CreateFixture(&rfd);
	  
	  b2RevoluteJointDef jointDef4,jointDef5,jointDef6;
	  jointDef4.bodyA=smallbox1;
	  jointDef5.bodyA=smallbox2;
	  jointDef6.bodyA=smallbox3;
	  jointDef4.bodyB=rod;
	  jointDef5.bodyB=rod;
	  jointDef6.bodyB=rod;
	  jointDef4.localAnchorA.Set(0,-7.5);
	  jointDef5.localAnchorA.Set(0,-7.5);
	  jointDef6.localAnchorA.Set(0,-7.5);
	  jointDef4.localAnchorB.Set(-45,0);
	  jointDef5.localAnchorB.Set(0,0);
	  jointDef6.localAnchorB.Set(45,0);
	  m_world->CreateJoint(&jointDef4);
	  m_world->CreateJoint(&jointDef5);
	  m_world->CreateJoint(&jointDef6);
	}
	{
		b2Body* sphere1;
	  b2CircleShape circle;
	  circle.m_radius=20.0;
	  b2FixtureDef ballfd1;
	  ballfd1.shape=&circle;
	  ballfd1.density = 5.0f;
      ballfd1.friction = 1.0f;
      ballfd1.restitution = 0.5f;
      ballfd1.filter.categoryBits=0x002;
      ballfd1.filter.maskBits=0x001|0x002;
      b2BodyDef ballbd1;
      ballbd1.type=b2_dynamicBody;
      ballbd1.position.Set(-40.f,80.f);
      sphere1=m_world->CreateBody(&ballbd1);
      sphere1->CreateFixture(&ballfd1);
  }
	
		
	}

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}

    
    
    
    
    
    
    
    
    
    
    
