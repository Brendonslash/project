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
    
	 {
	  b2Body* sphere;
	  b2CircleShape circle;
	  circle.m_radius=100.0;
	  b2FixtureDef ballfd;
	  ballfd.shape=&circle;
	  ballfd.density = 5.0f;
      ballfd.friction = 1.0f;
      ballfd.restitution = 0.5f;
      b2BodyDef ballbd;
      ballbd.type=b2_dynamicBody;
      ballbd.position.Set(0.0f,100.f);
      sphere=m_world->CreateBody(&ballbd);
      sphere->CreateFixture(&ballfd);
    }
    
    }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}

    
    
    
    
    
    
    
    
    
    
    
