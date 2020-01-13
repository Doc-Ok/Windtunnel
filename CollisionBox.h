/***********************************************************************
CollisionBox - Class to represent a rectangular box containing spheres
of a fixed radius interacting by fully elastic collisions.
Copyright (c) 2005-2020 Oliver Kreylos

This file is part of the Virtual Wind Tunnel package.

The Virtual Wind Tunnel package is free software; you can redistribute
it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Virtual Wind Tunnel package is distributed in the hope that it will
be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Virtual Wind Tunnel package; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#ifndef COLLISIONBOX_INCLUDED
#define COLLISIONBOX_INCLUDED

#include <Misc/Array.h>
#include <Misc/ChunkedArray.h>
#include <Misc/PriorityHeap.h>
#include <Geometry/ComponentArray.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/Box.h>

#define ACCUMULATE_PRESSURE 0

/* Forward declarations: */
namespace IO {
class File;
}

template <class ScalarParam,int dimensionParam>
class CollisionBox
	{
	/* Embedded classes: */
	public:
	typedef ScalarParam Scalar; // Data type for scalars
	static const int dimension=dimensionParam; // Dimension of collision box
	typedef Geometry::Point<Scalar,dimensionParam> Point; // Data type for points
	typedef Geometry::Vector<Scalar,dimensionParam> Vector; // Data type for points
	typedef Geometry::ComponentArray<Scalar,dimensionParam> Size; // Data type for sizes
	typedef Geometry::Box<Scalar,dimensionParam> Box; // Data type for axis-aligned boxes
	
	private:
	struct GridCell; // Forward declaration
	struct CollisionEvent; // Forward declaration
	
	public:
	class Particle // Class for fixed-radius spherical particles
		{
		friend class CollisionBox;
		friend class GridCell;
		friend class CollisionEvent;
		
		/* Elements: */
		private:
		Point position; // Current position of particle in collision box coordinates
		GridCell* cell; // Pointer to grid cell currently containing the particle
		Vector velocity; // Current velocity of particle in collision box coordinate
		Scalar timeStamp; // Time stamp of particle in current simulation step
		Particle* cellPred; // Pointer to particle's predecessor in same grid cell
		Particle* cellSucc; // Pointer to particle's successor in same grid cell
		
		/* Methods: */
		public:
		const Point& getPosition(void) const // Returns the particle's position
			{
			return position;
			}
		const Vector& getVelocity(void) const // Returns the particle's velocity
			{
			return velocity;
			}
		};
	
	typedef Misc::ChunkedArray<Particle> ParticleList; // Data type for lists of particles
	
	private:
	struct GridCell // Structure for grid cells containing particles
		{
		/* Elements: */
		public:
		Box boundaries; // Cell's bounding box (waste of space, really; should optimize this out)
		Particle* particlesHead; // Pointer to first particle in grid cell
		Particle* particlesTail; // Pointer to last particle in grid cell
		
		/* Constructors and destructors: */
		GridCell(void) // Creates uninitialized grid cell
			{
			};
		
		/* Methods: */
		void addParticle(Particle* newParticle) // Adds a particle to the grid cell
			{
			/* Link the new particle to the end of the cell's particle list: */
			newParticle->cell=this;
			newParticle->cellPred=particlesTail;
			newParticle->cellSucc=0;
			if(particlesTail!=0)
				particlesTail->cellSucc=newParticle;
			else
				particlesHead=newParticle;
			particlesTail=newParticle;
			};
		void removeParticle(Particle* removeParticle) // Removes a particle from the grid cell
			{
			/* Unlink the particle from the cell's list: */
			removeParticle->cell=0;
			if(removeParticle->cellPred!=0)
				removeParticle->cellPred->cellSucc=removeParticle->cellSucc;
			else
				particlesHead=removeParticle->cellSucc;
			if(removeParticle->cellSucc!=0)
				removeParticle->cellSucc->cellPred=removeParticle->cellPred;
			else
				particlesTail=removeParticle->cellPred;
			};
		};
	
	typedef Misc::Array<GridCell,dimensionParam> CellArray; // Data type for arrays of grid cells
	typedef typename CellArray::Index Index; // Data type for cell indices
	
	struct CollisionEvent // Structure to report potential collision events
		{
		/* Embedded classes: */
		enum CollisionType // Enumerated type for types of collisions
			{
			SphereWallCollision, // Spherical obstacle collides with a wall
			CellChange, // Particle moves from one grid cell into an adjacent one
			WallCollision, // Particle collides with wall
			SphereCollision, // Particle collides with spherical obstacle
			ParticleCollision // Two particles collide with each other
			};
		
		/* Elements: */
		public:
		CollisionType collisionType; // Type of this collision
		Scalar collisionTime; // Time at which this collision would occur
		Particle* particle1; // Pointer to first colliding particle
		Scalar timeStamp1; // Time stamp of first particle at time collision was detected
		int cellChangeDirection; // The index of the cell border crossed by the object
		Vector wallNormal; // Normal vector of wall involved in collision
		Particle* particle2; // Pointer to second colliding particle
		Scalar timeStamp2; // Time stamp of second particle at time collision was detected
		
		/* Constructors and destructors: */
		CollisionEvent(Scalar sCollisionTime,Scalar sphereTimeStamp,const Vector& sWallNormal) // Creates a spherical obstacle/wall collision event
			:collisionType(SphereWallCollision),collisionTime(sCollisionTime),
			 timeStamp1(sphereTimeStamp),
			 wallNormal(sWallNormal)
			{
			};
		CollisionEvent(Scalar sCollisionTime,Particle* sParticle1,int sCellChangeDirection) // Creates a particle cell change event
			:collisionType(CellChange),collisionTime(sCollisionTime),
			 particle1(sParticle1),timeStamp1(particle1->timeStamp),
			 cellChangeDirection(sCellChangeDirection)
			{
			};
		CollisionEvent(Scalar sCollisionTime,Particle* sParticle1,const Vector& sWallNormal) // Creates a particle/wall collision event
			:collisionType(WallCollision),collisionTime(sCollisionTime),
			 particle1(sParticle1),timeStamp1(particle1->timeStamp),
			 wallNormal(sWallNormal)
			{
			};
		CollisionEvent(Scalar sCollisionTime,Particle* sParticle1,Scalar sphereTimeStamp) // Creates a particle/spherical obstacle collision event
			:collisionType(SphereCollision),collisionTime(sCollisionTime),
			 particle1(sParticle1),timeStamp1(particle1->timeStamp),
			 timeStamp2(sphereTimeStamp)
			{
			};
		CollisionEvent(Scalar sCollisionTime,Particle* sParticle1,Particle* sParticle2) // Creates a particle/particle collision event
			:collisionType(ParticleCollision),collisionTime(sCollisionTime),
			 particle1(sParticle1),timeStamp1(particle1->timeStamp),
			 particle2(sParticle2),timeStamp2(particle2->timeStamp)
			{
			};
		
		/* Methods: */
		friend bool operator<=(const CollisionEvent& e1,const CollisionEvent& e2)
			{
			return e1.collisionTime<=e2.collisionTime;
			};
		};
	
	typedef Misc::PriorityHeap<CollisionEvent> CollisionQueue; // Data types for priority queues of collision events
	
	/* Elements: */
	Box boundaries; // Bounding box of entire collision box
	
	/* Collision grid definition: */
	Size cellSize; // Size of an individual cell
	int numCells[dimension]; // Number of interior cells
	CellArray cells; // Array of grid cells
	ssize_t directNeighborOffsets[dimension*2]; // Offsets between a cell and its direct neighbors
	int numNeighbors; // Number of direct neighbors of a cell (including the cell itself)
	ssize_t* neighborOffsets; // Pointer offsets between a cell and its neighbors
	int* cellChangeMasks; // Array of cell change direction masks for each neighbor
	
	/* Simulation parameters: */
	Scalar particleRadius,particleRadius2; // Radius and squared radius of all particles
	Vector gravity; // Gravity vector
	Scalar attenuation; // Factor by how much particles slow down over the course of one time unit; ==1: no slowdown
	
	/* List of particle states: */
	size_t numParticles; // Number of particles in the collision box
	ParticleList particles; // List of all particles in the collision box
	
	/* State for the spherical obstacle: */
	Point spherePosition; // Position of an additional spherical obstacle
	Vector sphereVelocity; // Velocity of spherical obstacle
	Scalar sphereRadius,sphereRadius2; // Radius and squared radius of spherical obstacle
	bool sphereOpen; // Flag if the sphere has a small opening on the bottom
	Scalar invSphereMass; // Inverse mass of the spherical obstacle
	Scalar sphereTimeStamp; // Time stamp of spherical obstacle in current time step
	
	/* Pre-computed interaction factors: */
	Scalar particleVelocityFactor; // Velocity factor when a particle collides with the spherical obstacle
	Scalar sphereVelocityFactor; // Velocity factor when a particle collides with the spherical obstacle
	
	#if ACCUMULATE_PRESSURE
	/* State to calculate gas pressure gradients: */
	int numPressureSlots; // Number of slots in the pressure accumulator
	Scalar pressureSlotSize; // Height of a pressure accumulator slot
	Scalar* pressureSlots; // Array of pressure accumulator slots
	Scalar pressureTime; // Total simulation time over which pressure has been accumulated
	#endif
	
	/* Private methods: */
	void queueSphereCollisions(Scalar timeStep,CollisionQueue& collisionQueue);
	void queueCollisionsInCell(GridCell* cell,Particle* particle1,Scalar timeStep,bool symmetric,Particle* otherParticle,CollisionQueue& collisionQueue);
	void queueCellChanges(Particle* particle,const Point& newPosition,Scalar timeStep,CollisionQueue& collisionQueue);
	void queueCollisions(Particle* particle1,Scalar timeStep,bool symmetric,Particle* otherParticle,CollisionQueue& collisionQueue);
	void queueCollisionsOnCellChange(Particle* particle1,Scalar timeStep,int cellChangeDirection,CollisionQueue& collisionQueue);
	void queueCollisionsWithSphere(Scalar timeStep,CollisionQueue& collisionQueue);
	
	/* Constructors and destructors: */
	public:
	CollisionBox(const Box& sBoundaries,Scalar sParticleRadius,Scalar sSphereRadius); // Creates box of given size, for particles of given radius
	CollisionBox(IO::File& file); // Creates box from the given state file
	~CollisionBox(void); // Destroys collision box and all particles
	
	/* Methods: */
	const Box& getBoundaries(void) const // Returns the collision box's boundaries
		{
		return boundaries;
		};
	Scalar getParticleRadius(void) const // Returns the radius of all particles
		{
		return particleRadius;
		}
	Scalar getAttenuation(void) const // Returns the attenuation factor
		{
		return attenuation;
		}
	Scalar getSphereRadius(void) const // Returns the radius of the spherical obstacle
		{
		return sphereRadius;
		}
	void setGravity(const Vector& newGravity); // Sets new gravity vector
	void setAttenuation(Scalar newAttenuation); // Sets new attenuation factor for particle velocities
	bool addParticle(const Point& newPosition,const Vector& newVelocity,bool sphereSolid =true); // Adds a new particle to the collision box; returns false if particle could not be added due to overlap with existing particles
	void setSphere(const Point& newPosition); // Directly moves the spherical obstacle to the given position, disregarding particle collisions
	void moveSphere(const Point& newPosition,Scalar timeStep); // Moves the spherical obstacle to the given position at the end of the next time step
	void setSphereOpen(bool newSphereOpen); // Sets whether the spherical obstacle has a small opening on the left side
	void setInverseSphereMass(Scalar newInvSphereMass); // Sets the inverse of the mass of the spherical obstacle
	void simulate(Scalar timeStep); // Advances simulation time by given time step
	const ParticleList& getParticles(void) const // Returns the list of particles
		{
		return particles;
		};
	const Point& getSphere(void) const // Returns the collision sphere's current position
		{
		return spherePosition;
		};
	Scalar calcAverageSpeed(const Box& queryBox) const; // Returns the average speed (magnitude of velocity) of all particles inside the given box
	void saveState(IO::File& file) const; // Writes the collision box's current state to the given file
	#if ACCUMULATE_PRESSURE
	void accumulatePressure(unsigned int newNumPressureSlots); // Starts accumulating gas pressure
	int getNumPressureSlots(void) const // Returns the number of pressure accumulation slots
		{
		return numPressureSlots;
		}
	const Scalar* getPressureSlots(void) const // Returns the pressure accumulator
		{
		return pressureSlots;
		}
	Scalar getPressureTime(void) const // Returns pressure accumulation time
		{
		return pressureTime;
		}
	#endif
	void fire(const Point& firePos,Scalar fireRadius,Scalar fireSpeed); // Sets a fire
	};

#ifndef COLLISIONBOX_IMPLEMENTATION
#include "CollisionBox.cpp"
#endif

#endif
