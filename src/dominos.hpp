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

#ifndef _DOMINOS_HPP_
#define _DOMINOS_HPP_

namespace cs296
{
  //! This is the class that sets up the Box2D simulation world
  //! Notice the public inheritance - why do we inherit the base_sim_t class?
  class dominos_t : public base_sim_t
  {
  public:
    
    dominos_t();

    void keyboard(unsigned char key);


    
    static base_sim_t* create()
    {
      return new dominos_t;
    }

    void step(settings_t* settings);
    //bool b;
    b2Body* ball[60];
    b2Fixture* fd1[60];

    /**
       * Ground
       * \li \b "b1" is the pointer to the class which contains the properties of Ground like its fixture,shape and state.
       * \li \b "shape" defines its shape,here it is a line segment so b2EdgeShape is used.
       * \li \b "bd" variable sets the body of Ground. 
       */
    b2Body* b1;  

    /**
       * Wheels
       * \li Here we define the wheels of our steam engine.
       * The maskBits when set to 0 make an object immune to
       * collisions. This is done because we just want the wheels to rotate
       * because of the shaft motion and not collide with them.
       * \li It has been made of type kinematic to test it when there is no steam
       * in which case a constant angular velocity has been applied to it so that
       * we can test the correctness of the system.
       * \li Revolute joints have been set at the center of the wheel
       * to make sure it rotates about the centers
       */
    b2Body* wheel1;
    b2Body* wheel2;
    b2Body* wheel3;
  
    
  /**
   *  wheelconnection
   * \li This wheelconnection is the rod connecting all the three wheels and
   * the connection is made using revolute joints
   * \li This rod make sures the wheels are at equal distance all through the simulation
   */
    b2Body* WheelConnection;
    
    /**
     *   Flycrank
     * 
     */
    b2Body* Flycrank;
    /**
     *   Flycrank rod
     * 
     */
    b2Body* Flycrankrod;
    /**
     *   supportflycrank
     * 
     */
    b2Body* SupportFlycrank;
    /**
     *   staticsupport
     * \li This is the static box which supports the whole flycrank system by a revolute joint 
     * This revolute joint is present between Supportflycrank and staticsupport.
     */
    b2Body* staticsupport;
    /**
     *   staticbars
     * \li These bars are the bars along which the lower piston slides in the simulation by using a prismatic joint.
     */
    b2Body* staticbardown;
    b2Body* staticbarup;
    /**
     *   lowepiston and upperpiston
     * \li These pistons are the which keeps the whole system in motion.
     * These pistons are effected by the steam pressure in such a way that the systems energy does not decrease.
     * For example when the pistons kinetic energy is low, that is when the piston are at antinodes then the steam is released 
     * The thermal energy of steam is converted into kinetic energy of piston which is transmitted to the wheels and so the train moves.
     */
    b2Body* lowerpiston;
    b2Body* upperpiston;
    
    /**
     *   longarm
     * \li This arm is connected to the middle wheel and the lower piston
     * With the help of this wheel kinetic energy of the piston is tranmitted to the wheel.
     * This is one of the two links present between piston system and wheel system.
     */
    b2Body* longarm;
    /**
     *   Unionlink
     * This is a rod having two joints.
     */
    b2Body* Unionlink;
    /**
     *   Combinationlever
     * The whole motion of the upperpiston is due to joint between upperpiston and Combinationlever.
     * 
     */
    
    b2Body* Combinationlever;
    
    /**
     *   Radiusbar
     * \li There is a line joint between Radiusbar and supportflycrank 
     * \li There is also a revolute joint between Radiusbar and supportflycrank.
     * when the switch gear system is activated then the current revolutejoint is destroyed and the radius bar moves along the line joint 
     * and when the switch gear system stops , a new revolute joint is created between radiusbar and supportflycrank.
     */
    b2Body* Radiusbar;
    b2Body* Liftinglink;
    /**
     *   armrest
     * \li armrest plays an important role in the switch gear system.
     * When switch gear is activated the armrest rotates resulting in the rotaition of lifting and reverse arms which causes the motion of radiusbar
     * up or down(depending on the current gear) along the line joint and so the upperpiston moves.
     */
    
    b2Body* armrest;
    b2Body* Liftingarm;
    b2Body* Reversearm;
    b2Body* Renchrod;
    b2Body* pistonrest;
    b2RevoluteJointDef ju;
    /**
     * This is the joint which is creted and destroyed during the switch gear process.
     */
    b2Joint* desjoin;
    b2Joint* desprism;
    /**
     *   Box
     * \li This is the whole container in which the upperpiston,lowerpiston are present.
     * This container contains variuos inlets and outlets through which steam comes in and goes out
     */
    b2Body* Box;
    /**
     *   train
     * This is just the box representing the static upper compartment in the simulation
     */
    b2Body* train;
    
    /**
     *   help objects
     * These bodies does not have any fixtures
     * These are present at two ends of radiusbar
     */
    b2Body* help1;
    b2Body* help2;
    
    
  b2RevoluteJointDef movj;
  
  b2PrismaticJointDef pjoint;
  /**
   * This is the line joint present between radius bar and suppertflycrank.
   */
  b2WheelJointDef wjoint;
  };  
}
  
#endif




