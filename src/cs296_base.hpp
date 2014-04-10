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


#ifndef _CS296BASE_HPP_
#define _CS296BASE_HPP_

#include "render.hpp"
#include <Box2D/Box2D.h>
#include <cstdlib>

#define	RAND_LIMIT 32767

namespace cs296
{

  //! What is the difference between a class and a struct in C++?
  class base_sim_t;
  struct settings_t;
  
  //! Why do we use a typedef
  typedef base_sim_t* sim_create_fcn(); 

  //! Simulation settings. Some can be controlled in the GUI.
  struct settings_t//! first struct
  {
    //! Notice the initialization of the class members in the constructor
    //! How is this happening?
    settings_t() :
      view_center(0.0f, 20.0f),
      hz(60.0f),
      velocity_iterations(8),
      position_iterations(3),
      draw_shapes(1),
      draw_joints(1),
      draw_AABBs(0),
      draw_pairs(0),
      draw_contact_points(0),
      draw_contact_normals(0),
      draw_contact_forces(0),
      draw_friction_forces(0),
      draw_COMs(0),
      draw_stats(0),
      draw_profile(0),
      enable_warm_starting(1),
      enable_continuous(1),
      enable_sub_stepping(0),
      pause(0),
      single_step(0)
    {}
    
    b2Vec2 view_center; //!< when the simulation is started it starts with the view_center as the center of the screen
    float32 hz;  //!< frequency with which frames refresh
    int32 velocity_iterations;//!< number of times velocity is corrected per frame.
    int32 position_iterations;//!< number of times position is corrected per frame.
    int32 draw_shapes;//!< shows the shapes of objects in the simulation
    int32 draw_joints;//!< draws the shapes of joints in the simulation.
    int32 draw_AABBs;//!< draws the bounding boxes along x-y of the objects in the simulation.
    int32 draw_pairs;//!< draws broad phase pairs.broad-phase collisions comes in handy when there are many bodies and their AABB's are used to find whether collision occurs.
    int32 draw_contact_points; //!< shows the contact points of various objects.
    int32 draw_contact_normals;//!< shows the values of contact forces along normal.
    int32 draw_contact_forces;//!< shows the values of contact forces.
    int32 draw_friction_forces;//!< shows the values of frictional forces.
    int32 draw_COMs; //!< shows the center of masses of objects in the simulation.
    int32 draw_stats;//!< shows the statistics at the beginning of the simulation.
    int32 draw_profile;//!< shows the profile of the simulation from the starting.
    int32 enable_warm_starting;//!< It is used when at beginning of the simulation no noticeable events take place.(like repeated events etc)
    int32 enable_continuous;//!< if steps are taken discretely then bodies tunnel each other so when time comes like at collision discreteness decreases 
    int32 enable_sub_stepping;//!< It is done to prevent overlapping of AABB's.simulation takes steps directly to the collision and then resolves it.
    int32 pause;//!< To pause the simulation at any moment of time 
    int32 single_step;//!< To view the simulation step by step ,per frame
  };
  
  struct sim_t//! second struct
  {
    const char *name;//!< name of the profile
    sim_create_fcn *create_fcn;//!< pointer to the complete profile

    sim_t(const char *_name, sim_create_fcn *_create_fcn): 
      name(_name), create_fcn(_create_fcn) {;}//!< defines the simulation with the given name and pointer
  };
  
  extern sim_t *sim;
  
  
  const int32 k_max_contact_points = 2048;  //!< numbber of contact points at some instant
  struct contact_point_t//!Third struct
  {
    b2Fixture* fixtureA; //!< fixtures A and B which are in contact
    b2Fixture* fixtureB;
    b2Vec2 normal;//!< all the contact forces along the normal vector
    b2Vec2 position;//!< position of the contact point
    b2PointState state;//!< state of the contact point,whether it a static like an hinge or dynamic like in rolling body
  };
  
  class base_sim_t : public b2ContactListener
  {//!< first class
  public:
    
    base_sim_t();//!< initializes the world

	inline b2World* get_World(void){
		return m_world;
	}
    //! Virtual destructors - amazing objects. Why are these necessary?
    virtual ~base_sim_t();//!< First destroys every object dependant on the current simulation and then destroys the complete simulation recursively.
    
    void set_text_line(int32 line) { m_text_line = line; }//!< initializes the text displayed on the simulation
    void draw_title(int x, int y, const char *string);//!< shows the title on simulation at x,y
    //!< most of the void fuctions with B2_NOT_USED does nothing,they are filled for the completeness of cases and avoid warnings 
    virtual void step(settings_t* settings);

    virtual void keyboard(unsigned char key) { B2_NOT_USED(key); }
    virtual void keyboard_up(unsigned char key) { B2_NOT_USED(key); }
    
    

    void shift_mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_up(const b2Vec2& p) { B2_NOT_USED(p); }
    void mouse_move(const b2Vec2& p) { B2_NOT_USED(p); }

    
    // Let derived tests know that a joint was destroyed.
    virtual void joint_destroyed(b2Joint* joint) { B2_NOT_USED(joint); }
    
    // Callbacks for derived classes.
    virtual void begin_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    virtual void end_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    virtual void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);
    virtual void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)
    {
      B2_NOT_USED(contact);
      B2_NOT_USED(impulse);
    }

  //!How are protected members different from private memebers of a class in C++ ?
  protected:

    //! What are Friend classes?
    friend class contact_listener_t;
    
    b2Body* m_ground_body;//!< poiter to the body of ground
    b2AABB m_world_AABB;  //!< bounding box of the world
    contact_point_t m_points[k_max_contact_points];//!< array of the contact points
    int32 m_point_count; //!< keeps a count to the number of contact points,if greater than the max_count stores the max_count 

    debug_draw_t m_debug_draw; //!< helps in debugging mode
    int32 m_text_line;
    b2World* m_world; //!< pointer to the defined world

    int32 m_step_count;  //!< keeps a count of the number of steps
    
    b2Profile m_max_profile;
    b2Profile m_total_profile;
  };
}

#endif
