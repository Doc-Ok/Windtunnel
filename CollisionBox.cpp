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

#define COLLISIONBOX_IMPLEMENTATION

#include "CollisionBox.h"

#include <iostream>
#include <Misc/SizedTypes.h>
#include <IO/File.h>
#include <Math/Math.h>

#define CENTRAL_GRAVITY 1
#define GRAVITY_SQUARELAW 1

/*****************************
Methods of class CollisionBox:
*****************************/

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::queueSphereCollisions(
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar timeStep,
	typename CollisionBox<ScalarParam,dimensionParam>::CollisionQueue& collisionQueue)
	{
	/* Calculate the spherical obstacle's position at the end of this time step: */
	Point newSpherePosition=spherePosition+sphereVelocity*(timeStep-sphereTimeStamp);
	
	Vector wallNormal;
	Scalar collisionTime=timeStep;
	for(int dim=0;dim<dimension;++dim)
		{
		/* Check if the spherical obstacle leaves the domain: */
		if(newSpherePosition[dim]<boundaries.min[dim]+sphereRadius)
			{
			/* Calculate the event time: */
			Scalar ct=(boundaries.min[dim]+sphereRadius-spherePosition[dim])/sphereVelocity[dim]+sphereTimeStamp;
			if(ct>sphereTimeStamp&&collisionTime>ct)
				{
				wallNormal=Vector::zero;
				wallNormal[dim]=Scalar(1);
				collisionTime=ct;
				}
			}
		else if(newSpherePosition[dim]>boundaries.max[dim]-sphereRadius)
			{
			/* Calculate the event time: */
			Scalar ct=(boundaries.max[dim]-sphereRadius-spherePosition[dim])/sphereVelocity[dim]+sphereTimeStamp;
			if(ct>sphereTimeStamp&&collisionTime>ct)
				{
				wallNormal=Vector::zero;
				wallNormal[dim]=Scalar(-1);
				collisionTime=ct;
				}
			}
		}
	
	if(collisionTime<timeStep)
		collisionQueue.insert(CollisionEvent(collisionTime,sphereTimeStamp,wallNormal));
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::queueCollisionsInCell(
	typename CollisionBox<ScalarParam,dimensionParam>::GridCell* cell,
	typename CollisionBox<ScalarParam,dimensionParam>::Particle* particle1,
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar timeStep,
	bool symmetric,
	typename CollisionBox<ScalarParam,dimensionParam>::Particle* otherParticle,
	typename CollisionBox<ScalarParam,dimensionParam>::CollisionQueue& collisionQueue)
	{
	/* Calculate all intersections between two particles: */
	for(Particle* particle2=cell->particlesHead;particle2!=0;particle2=particle2->cellSucc)
		if(particle2!=particle1&&particle2!=otherParticle&&(symmetric||particle2>particle1))
			{
			/* Calculate any possible intersection time between the two particles: */
			Vector d=particle1->position-particle2->position;
			d-=particle1->velocity*particle1->timeStamp;
			d+=particle2->velocity*particle2->timeStamp;
			Vector vd=particle1->velocity-particle2->velocity;
			Scalar vd2=Geometry::sqr(vd);
			if(vd2>Scalar(0)) // Are the two particles' velocities different?
				{
				/* Solve the quadratic equation determining possible collisions: */
				Scalar ph=(d*vd)/vd2;
				Scalar q=(Geometry::sqr(d)-Scalar(4)*particleRadius2)/vd2;
				Scalar det=Math::sqr(ph)-q;
				if(det>=Scalar(0)) // Are there any solutions?
					{
					/* Calculate the first solution (only that can be valid): */
					Scalar collisionTime=-ph-Math::sqrt(det);

					/* If the collision is valid, i.e., occurs past the last update of both particles, queue it: */
					if(collisionTime>particle1->timeStamp&&collisionTime>particle2->timeStamp&&collisionTime<=timeStep)
						collisionQueue.insert(CollisionEvent(collisionTime,particle1,particle2));
					}
				}
			}
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::queueCellChanges(
	typename CollisionBox<ScalarParam,dimensionParam>::Particle* particle,
	const typename CollisionBox<ScalarParam,dimensionParam>::Point& newPosition,
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar timeStep,
	typename CollisionBox<ScalarParam,dimensionParam>::CollisionQueue& collisionQueue)
	{
	/* Check for crossing of any cell borders: */
	GridCell* cell=particle->cell;
	Scalar cellChangeTime=timeStep;
	int cellChangeDirection=-1;
	for(int i=0;i<dimension;++i)
		{
		if(newPosition[i]<cell->boundaries.min[i])
			{
			Scalar collisionTime=particle->timeStamp+(cell->boundaries.min[i]-particle->position[i])/particle->velocity[i];
			if(cellChangeTime>collisionTime)
				{
				cellChangeTime=collisionTime;
				cellChangeDirection=2*i+0;
				}
			}
		else if(newPosition[i]>cell->boundaries.max[i])
			{
			Scalar collisionTime=particle->timeStamp+(cell->boundaries.max[i]-particle->position[i])/particle->velocity[i];
			if(cellChangeTime>collisionTime)
				{
				cellChangeTime=collisionTime;
				cellChangeDirection=2*i+1;
				}
			}
		}
	if(cellChangeDirection>=0)
		collisionQueue.insert(CollisionEvent(cellChangeTime,particle,cellChangeDirection));
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::queueCollisions(
	typename CollisionBox<ScalarParam,dimensionParam>::Particle* particle1,
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar timeStep,
	bool symmetric,
	typename CollisionBox<ScalarParam,dimensionParam>::Particle* otherParticle,
	typename CollisionBox<ScalarParam,dimensionParam>::CollisionQueue& collisionQueue)
	{
	/* Calculate the particle's position at the end of this time step: */
	Point newPosition=particle1->position+particle1->velocity*(timeStep-particle1->timeStamp);
	
	/* Check for crossing of cell borders: */
	queueCellChanges(particle1,newPosition,timeStep,collisionQueue);
	
	/* Check for collisions with any of the collision box's walls: */
	for(int i=0;i<dimension;++i)
		{
		if(newPosition[i]<boundaries.min[i]+particleRadius)
			{
			Scalar collisionTime=particle1->timeStamp+(boundaries.min[i]+particleRadius-particle1->position[i])/particle1->velocity[i];
			if(collisionTime<particle1->timeStamp)
				collisionTime=particle1->timeStamp;
			else if(collisionTime>timeStep)
				collisionTime=timeStep;
			Vector wallNormal=Vector::zero;
			wallNormal[i]=Scalar(1);
			collisionQueue.insert(CollisionEvent(collisionTime,particle1,wallNormal));
			}
		else if(newPosition[i]>boundaries.max[i]-particleRadius)
			{
			Scalar collisionTime=particle1->timeStamp+(boundaries.max[i]-particleRadius-particle1->position[i])/particle1->velocity[i];
			if(collisionTime<particle1->timeStamp)
				collisionTime=particle1->timeStamp;
			else if(collisionTime>timeStep)
				collisionTime=timeStep;
			Vector wallNormal=Vector::zero;
			wallNormal[i]=Scalar(-1);
			collisionQueue.insert(CollisionEvent(collisionTime,particle1,wallNormal));
			}
		}
	
	/* Check for collision with the spherical obstacle: */
	Vector d=particle1->position-spherePosition;
	d-=particle1->velocity*particle1->timeStamp;
	d+=sphereVelocity*sphereTimeStamp;
	Vector vd=particle1->velocity-sphereVelocity;
	Scalar vd2=Geometry::sqr(vd);
	if(vd2>Scalar(0)) // Are the particle's and the sphere's velocities different?
		{
		Scalar collisionTime(0);
		
		/* Check if the particle is inside or outside the sphere: */
		Scalar d2=d.sqr();
		if(d2>=sphereRadius2)
			{
			/* Solve the quadratic equation determining possible collisions: */
			Scalar ph=(d*vd)/vd2;
			Scalar q=(d2-Math::sqr(sphereRadius+particleRadius))/vd2;
			Scalar det=Math::sqr(ph)-q;
			if(det>=Scalar(0)) // Are there any solutions?
				{
				/* Calculate the first solution (only that can be valid): */
				collisionTime=-ph-Math::sqrt(det);
				}
			}
		else
			{
			/* Solve the quadratic equation determining possible collisions: */
			Scalar ph=(d*vd)/vd2;
			Scalar q=(d2-Math::sqr(sphereRadius-particleRadius))/vd2;
			Scalar det=Math::sqr(ph)-q;
			if(det>=Scalar(0)) // Are there any solutions?
				{
				/* Calculate the second solution (only that can be valid): */
				collisionTime=-ph+Math::sqrt(det);
				}
			}
		
		/* If the collision is valid, i.e., occurs past the last update of both the particle and the sphere, queue it: */
		if(collisionTime>particle1->timeStamp&&collisionTime>sphereTimeStamp&&collisionTime<=timeStep)
			{
			/* Check if the sphere is open: */
			if(sphereOpen)
				{
				/* Calculate the collision offset: */
				d+=(particle1->velocity-sphereVelocity)*collisionTime;
				if(d[1]>=Scalar(0)||Math::abs(d[0])>=sphereRadius*Scalar(0.075))
					collisionQueue.insert(CollisionEvent(collisionTime,particle1,sphereTimeStamp));
				}
			else
				collisionQueue.insert(CollisionEvent(collisionTime,particle1,sphereTimeStamp));
			}
		}
	
	/* Check for collisions with any other particle: */
	GridCell* baseCell=particle1->cell;
	for(int i=0;i<numNeighbors;++i)
		{
		GridCell* cell=baseCell+neighborOffsets[i];
		queueCollisionsInCell(cell,particle1,timeStep,symmetric,otherParticle,collisionQueue);
		}
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::queueCollisionsOnCellChange(
	typename CollisionBox<ScalarParam,dimensionParam>::Particle* particle,
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar timeStep,
	int cellChangeDirection,
	typename CollisionBox<ScalarParam,dimensionParam>::CollisionQueue& collisionQueue)
	{
	/* Calculate the particle's position at the end of this time step: */
	Point newPosition=particle->position+particle->velocity*(timeStep-particle->timeStamp);
	
	/* Check for crossing of cell borders: */
	queueCellChanges(particle,newPosition,timeStep,collisionQueue);
	
	/* Check for collision with any other particle: */
	GridCell* baseCell=particle->cell;
	for(int i=0;i<numNeighbors;++i)
		if(cellChangeMasks[i]&(1<<cellChangeDirection))
			{
			GridCell* cell=baseCell+neighborOffsets[i];
			queueCollisionsInCell(cell,particle,timeStep,true,0,collisionQueue);
			}
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::queueCollisionsWithSphere(
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar timeStep,
	typename CollisionBox<ScalarParam,dimensionParam>::CollisionQueue& collisionQueue)
	{
	/* Check all particles: */
	for(typename ParticleList::iterator pIt=particles.begin();pIt!=particles.end();++pIt)
		{
		/* Check for collision with the spherical obstacle: */
		Vector d=pIt->position-spherePosition;
		d-=pIt->velocity*pIt->timeStamp;
		d+=sphereVelocity*sphereTimeStamp;
		Vector vd=pIt->velocity-sphereVelocity;
		Scalar vd2=Geometry::sqr(vd);
		if(vd2>Scalar(0)) // Are the particle's and the sphere's velocities different?
			{
			Scalar collisionTime(0);
			
			/* Check if the particle is inside or outside the sphere: */
			Scalar d2=d.sqr();
			if(d2>=sphereRadius2)
				{
				/* Solve the quadratic equation determining possible collisions: */
				Scalar ph=(d*vd)/vd2;
				Scalar q=(d2-Math::sqr(sphereRadius+particleRadius))/vd2;
				Scalar det=Math::sqr(ph)-q;
				if(det>=Scalar(0)) // Are there any solutions?
					{
					/* Calculate the first solution (only that can be valid): */
					collisionTime=-ph-Math::sqrt(det);
					}
				}
			else
				{
				/* Solve the quadratic equation determining possible collisions: */
				Scalar ph=(d*vd)/vd2;
				Scalar q=(d2-Math::sqr(sphereRadius-particleRadius))/vd2;
				Scalar det=Math::sqr(ph)-q;
				if(det>=Scalar(0)) // Are there any solutions?
					{
					/* Calculate the second solution (only that can be valid): */
					collisionTime=-ph+Math::sqrt(det);
					}
				}
			
			/* If the collision is valid, i.e., occurs past the last update of both the particle and the sphere, queue it: */
			if(collisionTime>pIt->timeStamp&&collisionTime>sphereTimeStamp&&collisionTime<=timeStep)
				{
				/* Check if the sphere is open: */
				if(sphereOpen)
					{
					/* Calculate the collision offset: */
					d+=(pIt->velocity-sphereVelocity)*collisionTime;
					if(d[1]>=Scalar(0)||Math::abs(d[0])>=sphereRadius*Scalar(0.075))
						collisionQueue.insert(CollisionEvent(collisionTime,&*pIt,sphereTimeStamp));
					}
				else
					collisionQueue.insert(CollisionEvent(collisionTime,&*pIt,sphereTimeStamp));
				}
			}
		}
	}

template <class ScalarParam,int dimensionParam>
inline
CollisionBox<ScalarParam,dimensionParam>::CollisionBox(
	const typename CollisionBox<ScalarParam,dimensionParam>::Box& sBoundaries,
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar sParticleRadius,
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar sSphereRadius)
	:boundaries(sBoundaries),
	 numNeighbors(0),neighborOffsets(0),cellChangeMasks(0),
	 particleRadius(sParticleRadius),particleRadius2(Math::sqr(particleRadius)),
	 gravity(Vector::zero),
	 attenuation(1),
	 numParticles(0),
	 spherePosition(Point::origin),
	 sphereVelocity(Vector::zero),
	 sphereOpen(false),
	 sphereRadius(sSphereRadius),sphereRadius2(Math::sqr(sphereRadius)),
	 invSphereMass(0),sphereTimeStamp(0),
	 particleVelocityFactor(2),sphereVelocityFactor(0)
	 #if ACCUMULATE_PRESSURE
	 ,
	 numPressureSlots(0),pressureSlotSize(0),pressureSlots(0),pressureTime(0)
	 #endif
	{
	/* Calculate optimal number of cells and cell sizes: */
	Index numOuterCells;
	for(int i=0;i<dimension;++i)
		{
		numCells[i]=int(Math::floor(boundaries.getSize(i)/(particleRadius*Scalar(2.5))));
		cellSize[i]=boundaries.getSize(i)/Scalar(numCells[i]);
		numOuterCells[i]=numCells[i]+2; // Create a layer of "ghost cells" in all directions
		}
	
	/* Create the cell array: */
	cells.resize(numOuterCells);
	for(Index index(0);index[0]<numOuterCells[0];cells.preInc(index))
		{
		/* Initialize the cell: */
		Point min,max;
		for(int i=0;i<dimension;++i)
			{
			min[i]=boundaries.min[i]+cellSize[i]*Scalar(index[i]-1);
			max[i]=boundaries.min[i]+cellSize[i]*Scalar(index[i]-0);
			}
		cells(index).boundaries=Box(min,max);
		cells(index).particlesHead=0;
		cells(index).particlesTail=0;
		}
	
	/* Initialize the direct neighbor offsets: */
	for(int i=0;i<dimension;++i)
		{
		directNeighborOffsets[2*i+0]=-cells.getIncrement(i);
		directNeighborOffsets[2*i+1]=cells.getIncrement(i);
		}
	
	/* Initialize the neighbor array: */
	numNeighbors=1;
	for(int i=0;i<dimension;++i)
		numNeighbors*=3;
	neighborOffsets=new ssize_t[numNeighbors];
	cellChangeMasks=new int[numNeighbors];
	Index minBound;
	Index maxBound;
	for(int i=0;i<dimension;++i)
		{
		minBound[i]=-1;
		maxBound[i]=2;
		}
	int neighborIndex=0;
	for(Index index=minBound;index[0]<maxBound[0];index.preInc(minBound,maxBound),++neighborIndex)
		{
		neighborOffsets[neighborIndex]=0;
		cellChangeMasks[neighborIndex]=0x0;
		for(int i=0;i<dimension;++i)
			{
			neighborOffsets[neighborIndex]+=cells.getIncrement(i)*index[i];
			if(index[i]==-1)
				cellChangeMasks[neighborIndex]|=1<<(2*i+0);
			else if(index[i]==1)
				cellChangeMasks[neighborIndex]|=1<<(2*i+1);
			}
		}
	
	/* Position the spherical obstacle: */
	spherePosition[0]=boundaries.min[0]-sphereRadius-Scalar(10);
	for(int i=1;i<dimension;++i)
		spherePosition[i]=Math::div2(boundaries.min[i]+boundaries.max[i]);
	// sphereVelocity[0]=Scalar(5);
	}

template <class ScalarParam,int dimensionParam>
inline
CollisionBox<ScalarParam,dimensionParam>::CollisionBox(
	IO::File& file)
	:neighborOffsets(0),cellChangeMasks(0)
	 #if ACCUMULATE_PRESSURE
	 ,
	 numPressureSlots(0),pressureSlotSize(0),pressureSlots(0),pressureTime(0)
	 #endif
	{
	/* Read the collision box vector space dimension and scalar type for compatibility checks: */
	int fileDimension=file.read<Misc::UInt32>();
	size_t fileScalarSize=file.read<Misc::UInt32>();
	if(fileDimension!=dimension||fileScalarSize!=sizeof(Scalar))
		throw std::runtime_error("CollisionBox::CollisionBox: File has incompatible format");
	
	/* Read the collision box boundaries: */
	file.read(boundaries.min.getComponents(),dimension);
	file.read(boundaries.max.getComponents(),dimension);
	
	/* Read the cell size and number of cells: */
	file.read(cellSize.getComponents(),dimension);
	file.read(numCells,dimension);
	
	#if 0
	
	/* Grow the box vertically: */
	numCells[1]*=2;
	boundaries.max[1]=boundaries.min[1]+(boundaries.max[1]-boundaries.min[1])*Scalar(2);
	
	#endif
	
	/* Create the cell array: */
	Index numOuterCells;
	for(int i=0;i<dimension;++i)
		numOuterCells[i]=numCells[i]+2;
	cells.resize(numOuterCells);
	for(Index index(0);index[0]<numOuterCells[0];cells.preInc(index))
		{
		/* Initialize the cell: */
		Point min,max;
		for(int i=0;i<dimension;++i)
			{
			min[i]=boundaries.min[i]+cellSize[i]*Scalar(index[i]-1);
			max[i]=boundaries.min[i]+cellSize[i]*Scalar(index[i]-0);
			}
		cells(index).boundaries=Box(min,max);
		cells(index).particlesHead=0;
		cells(index).particlesTail=0;
		}
	
	/* Initialize the direct neighbor offsets: */
	for(int i=0;i<dimension;++i)
		{
		directNeighborOffsets[2*i+0]=-cells.getIncrement(i);
		directNeighborOffsets[2*i+1]=cells.getIncrement(i);
		}
	
	/* Initialize the neighbor array: */
	numNeighbors=1;
	for(int i=0;i<dimension;++i)
		numNeighbors*=3;
	neighborOffsets=new ssize_t[numNeighbors];
	cellChangeMasks=new int[numNeighbors];
	Index minBound;
	Index maxBound;
	for(int i=0;i<dimension;++i)
		{
		minBound[i]=-1;
		maxBound[i]=2;
		}
	int neighborIndex=0;
	for(Index index=minBound;index[0]<maxBound[0];index.preInc(minBound,maxBound),++neighborIndex)
		{
		neighborOffsets[neighborIndex]=0;
		cellChangeMasks[neighborIndex]=0x0;
		for(int i=0;i<dimension;++i)
			{
			neighborOffsets[neighborIndex]+=cells.getIncrement(i)*index[i];
			if(index[i]==-1)
				cellChangeMasks[neighborIndex]|=1<<(2*i+0);
			else if(index[i]==1)
				cellChangeMasks[neighborIndex]|=1<<(2*i+1);
			}
		}
	
	/* Read simulation parameters: */
	file.read(particleRadius);
	particleRadius2=Math::sqr(particleRadius);
	file.read(gravity.getComponents(),dimension);
	file.read(attenuation);
	
	/* Read the state of the spherical obstacle: */
	file.read(spherePosition.getComponents(),dimension);
	file.read(sphereVelocity.getComponents(),dimension);
	file.read(sphereRadius);
	sphereRadius2=Math::sqr(sphereRadius);
	sphereOpen=file.read<Misc::UInt8>()!=0;
	file.read(invSphereMass);
	particleVelocityFactor=Scalar(2)/(invSphereMass+Scalar(1));
	sphereVelocityFactor=(Scalar(2)*invSphereMass)/(invSphereMass+Scalar(1));
	sphereTimeStamp=Scalar(0);
	
	/* Read the states of all particles: */
	size_t fileNumParticles=size_t(file.read<Misc::UInt32>());
	// std::cout<<"Reading "<<fileNumParticles<<" particles"<<std::endl;
	for(size_t i=0;i<fileNumParticles;++i)
		{
		Point pos;
		file.read(pos.getComponents(),dimension);
		Vector vel;
		file.read(vel.getComponents(),dimension);
		if(!addParticle(pos,vel))
			std::cout<<"Warning: Unable to add particle from state file"<<std::endl;
		}
	}

template <class ScalarParam,int dimensionParam>
inline
CollisionBox<ScalarParam,dimensionParam>::~CollisionBox(
	void)
	{
	delete[] neighborOffsets;
	delete[] cellChangeMasks;
	
	#if ACCUMULATE_PRESSURE
	delete[] pressureSlots;
	#endif
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::setAttenuation(
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar newAttenuation)
	{
	attenuation=newAttenuation;
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::setGravity(
	const typename CollisionBox<ScalarParam,dimensionParam>::Vector& newGravity)
	{
	gravity=newGravity;
	}

template <class ScalarParam,int dimensionParam>
inline
bool
CollisionBox<ScalarParam,dimensionParam>::addParticle(
	const typename CollisionBox<ScalarParam,dimensionParam>::Point& newPosition,
	const typename CollisionBox<ScalarParam,dimensionParam>::Vector& newVelocity,
	bool sphereSolid)
	{
	/* Find the cell containing the new particle: */
	Point newP=newPosition;
	Index cellIndex;
	for(int i=0;i<dimension;++i)
		{
		if(newP[i]<boundaries.min[i]+particleRadius)
			newP[i]=boundaries.min[i]+particleRadius;
		else if(newP[i]>boundaries.max[i]-particleRadius)
			newP[i]=boundaries.max[i]-particleRadius;
		cellIndex[i]=int(Math::floor((newP[i]-boundaries.min[i])/cellSize[i]))+1;
		}
	GridCell* cell=cells.getAddress(cellIndex);
	
	/* Check if there is room to add the new particle: */
	for(int i=0;i<numNeighbors;++i)
		{
		GridCell* neighborCell=cell+neighborOffsets[i];
		
		for(Particle* pPtr=neighborCell->particlesHead;pPtr!=0;pPtr=pPtr->cellSucc)
			{
			Scalar dist2=Geometry::sqrDist(pPtr->position,newP);
			if(dist2<=Scalar(4)*particleRadius2)
				return false; // Could not add the particle
			}
		}
	
	/* Check if the particle overlaps the sphere: */
	Scalar sphereDist2=Geometry::sqrDist(newPosition,spherePosition);
	if(sphereDist2<=Math::sqr(sphereRadius+particleRadius)&&(sphereSolid||sphereDist2>=Math::sqr(sphereRadius+particleRadius)))
		return false; // Could not add the particle
	
	/* Add a new particle to the particle list: */
	particles.push_back(Particle());
	Particle& p=particles.back();
	
	/* Initialize the new particle: */
	p.position=newPosition;
	p.velocity=newVelocity;
	p.timeStamp=Scalar(0);
	cell->addParticle(&p);
	
	++numParticles;
	return true; // Particle succesfully added
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::setSphere(
	const typename CollisionBox<ScalarParam,dimensionParam>::Point& newPosition)
	{
	/* Set the sphere's position and reset its velocity: */
	spherePosition=newPosition;
	sphereVelocity=Vector::zero;
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::moveSphere(
	const typename CollisionBox<ScalarParam,dimensionParam>::Point& newPosition,
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar timeStep)
	{
	/* Calculate the sphere's velocity for this time step: */
	sphereVelocity=(newPosition-spherePosition)/timeStep;
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::setSphereOpen(
	bool newSphereOpen)
	{
	sphereOpen=newSphereOpen;
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::setInverseSphereMass(
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar newInvSphereMass)
	{
	/* Set the inverse sphere mass: */
	invSphereMass=newInvSphereMass;
	
	/* Update the particle and sphere collision velocity factors: */
	particleVelocityFactor=Scalar(2)/(invSphereMass+Scalar(1));
	sphereVelocityFactor=(Scalar(2)*invSphereMass)/(invSphereMass+Scalar(1));
	}

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::simulate(
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar timeStep)
	{
	/* Initialize the collision queue: */
	CollisionQueue collisionQueue(numParticles*dimension);
	for(typename ParticleList::iterator pIt=particles.begin();pIt!=particles.end();++pIt)
		queueCollisions(&(*pIt),timeStep,false,0,collisionQueue);
	if(invSphereMass!=Scalar(0))
		queueSphereCollisions(timeStep,collisionQueue);
	
	/* Update all particles' positions and handle all collisions: */
	while(!collisionQueue.isEmpty())
		{
		/* Get the next collision from the queue: */
		CollisionEvent nc=collisionQueue.getSmallest();
		collisionQueue.removeSmallest();
		
		/* Handle the collision if it is still valid: */
		switch(nc.collisionType)
			{
			case CollisionEvent::SphereWallCollision:
				if(sphereTimeStamp==nc.timeStamp1)
					{
					/* Bounce the spherical obstacle off the wall: */
					spherePosition+=sphereVelocity*(nc.collisionTime-nc.timeStamp1);
					sphereTimeStamp=nc.collisionTime;
					Vector dv=(Scalar(2)*(nc.wallNormal*sphereVelocity))*nc.wallNormal;
					sphereVelocity-=dv;
					
					/* Re-calculate all collisions of the spherical obstacle: */
					queueCollisionsWithSphere(timeStep,collisionQueue);
					queueSphereCollisions(timeStep,collisionQueue);
					}
				break;
			
			case CollisionEvent::CellChange:
				if(nc.particle1->timeStamp==nc.timeStamp1)
					{
					/* Let the particle cross into the next grid cell: */
					GridCell* cell=nc.particle1->cell;
					cell->removeParticle(nc.particle1);
					cell+=directNeighborOffsets[nc.cellChangeDirection];
					cell->addParticle(nc.particle1);
					
					/* Re-calculate all the particle's collisions: */
					queueCollisionsOnCellChange(nc.particle1,timeStep,nc.cellChangeDirection,collisionQueue);
					}
				break;
			
			case CollisionEvent::WallCollision:
				if(nc.particle1->timeStamp==nc.timeStamp1)
					{
					/* Bounce the particle off the wall: */
					nc.particle1->position+=nc.particle1->velocity*(nc.collisionTime-nc.timeStamp1);
					nc.particle1->timeStamp=nc.collisionTime;
					#if 1
					Vector dv=(Scalar(2)*(nc.wallNormal*nc.particle1->velocity))*nc.wallNormal;
					nc.particle1->velocity-=dv;
					#else
					/* Create convection by making the floor "hot" and the ceiling "cold": */
					Vector dv=(nc.wallNormal*nc.particle1->velocity)*nc.wallNormal;
					nc.particle1->velocity-=dv;
					if(nc.wallNormal[1]<Scalar(0))
						nc.particle1->velocity-=dv*Scalar(0.1);
					else if(nc.wallNormal[1]>Scalar(0))
						nc.particle1->velocity-=dv*Scalar(1.05);
					else
						nc.particle1->velocity-=dv;
					#endif
					
					#if ACCUMULATE_PRESSURE
					if(numPressureSlots>0&&nc.wallNormal[1]==Scalar(0))
						{
						/* Accumulate pressure: */
						unsigned int psIndex=(unsigned int)(Math::floor(nc.particle1->position[1]/pressureSlotSize));
						if(nc.wallNormal[0]>Scalar(0))
							pressureSlots[psIndex*2+0]-=dv[0];
						else
							pressureSlots[psIndex*2+1]+=dv[0];
						}
					#endif
					
					/* Re-calculate all the particle's collisions: */
					queueCollisions(nc.particle1,timeStep,true,0,collisionQueue);
					}
				break;
			
			case CollisionEvent::SphereCollision:
				if(nc.particle1->timeStamp==nc.timeStamp1&&sphereTimeStamp==nc.timeStamp2)
					{
					/* Bounce the particle off the sphere: */
					nc.particle1->position+=nc.particle1->velocity*(nc.collisionTime-nc.timeStamp1);
					nc.particle1->timeStamp=nc.collisionTime;
					Point sp=spherePosition+sphereVelocity*(nc.collisionTime-nc.timeStamp2);
					Vector d=sp-nc.particle1->position;
					Scalar dLen2=Geometry::sqr(d);
					Vector v1=d*((nc.particle1->velocity*d)/dLen2);
					Vector v2=d*((sphereVelocity*d)/dLen2);
					nc.particle1->velocity+=(v2-v1)*particleVelocityFactor;
					
					/* Re-calculate all collisions of the particle: */
					queueCollisions(nc.particle1,timeStep,true,0,collisionQueue);
					
					if(invSphereMass>Scalar(0))
						{
						/* Update the position and velocity of the spherical obstacle: */
						spherePosition=sp;
						sphereTimeStamp=nc.collisionTime;
						sphereVelocity+=(v1-v2)*sphereVelocityFactor;
						
						/* Re-calculate all collisions of the spherical obstacle: */
						queueCollisionsWithSphere(timeStep,collisionQueue);
						queueSphereCollisions(timeStep,collisionQueue);
						}
					}
				break;
			
			case CollisionEvent::ParticleCollision:
				if(nc.particle1->timeStamp==nc.timeStamp1&&nc.particle2->timeStamp==nc.timeStamp2)
					{
					/* Bounce the two particles off each other: */
					nc.particle1->position+=nc.particle1->velocity*(nc.collisionTime-nc.timeStamp1);
					nc.particle1->timeStamp=nc.collisionTime;
					nc.particle2->position+=nc.particle2->velocity*(nc.collisionTime-nc.timeStamp2);
					nc.particle2->timeStamp=nc.collisionTime;
					Vector d=nc.particle2->position-nc.particle1->position;
					Scalar dLen2=Geometry::sqr(d);
					Vector v1=d*((nc.particle1->velocity*d)/dLen2);
					Vector v2=d*((nc.particle2->velocity*d)/dLen2);
					Vector dv=v2-v1;
					nc.particle1->velocity+=dv;
					nc.particle2->velocity-=dv;
					
					/* Re-calculate all collisions of both particles: */
					queueCollisions(nc.particle1,timeStep,true,nc.particle2,collisionQueue);
					queueCollisions(nc.particle2,timeStep,true,nc.particle1,collisionQueue);
					}
				break;
			}
		}
	
	/* Update all particles to the end of the timestep: */
	#if CENTRAL_GRAVITY
	Scalar g=gravity.mag();
	#else
	Vector dv=gravity*timeStep; // Velocity change in this time step due to gravity
	#endif
	if(attenuation!=Scalar(1))
		{
		Scalar att=Math::pow(attenuation,timeStep); // Scale attenuation factor for this time step
		for(typename ParticleList::iterator pIt=particles.begin();pIt!=particles.end();++pIt)
			{
			/* Advance the particle's state: */
			#if CENTRAL_GRAVITY
			Vector dp=pIt->velocity*(timeStep-pIt->timeStamp);
			Vector gDir=spherePosition-(pIt->position+dp*Scalar(0.5));
			pIt->position+=dp;
			#if GRAVITY_SQUARELAW
			Scalar gd2=gDir.sqr();
			pIt->velocity+=gDir*(timeStep*g/(gd2*Math::sqrt(gd2)));
			#else
			pIt->velocity+=gDir*(timeStep*g/gDir.mag());
			#endif
			#else
			pIt->position+=pIt->velocity*(timeStep-pIt->timeStamp);
			pIt->velocity+=dv;
			#endif
			pIt->timeStamp=Scalar(0);
			
			/* Attenuate the particle's velocity: */
			pIt->velocity*=att;
			}
		}
	else
		{
		for(typename ParticleList::iterator pIt=particles.begin();pIt!=particles.end();++pIt)
			{
			/* Advance the particle's state: */
			#if CENTRAL_GRAVITY
			Vector dp=pIt->velocity*(timeStep-pIt->timeStamp);
			Vector gDir=spherePosition-(pIt->position+dp*Scalar(0.5));
			pIt->position+=dp;
			#if GRAVITY_SQUARELAW
			Scalar gd2=gDir.sqr();
			pIt->velocity+=gDir*(timeStep*g/(gd2*Math::sqrt(gd2)));
			#else
			pIt->velocity+=gDir*(timeStep*g/gDir.mag());
			#endif
			#else
			pIt->position+=pIt->velocity*(timeStep-pIt->timeStamp);
			pIt->velocity+=dv;
			#endif
			pIt->timeStamp=Scalar(0);
			}
		}
	
	/* Update the collision sphere to the end of the time step: */
	spherePosition+=sphereVelocity*(timeStep-sphereTimeStamp);
	#if !CENTRAL_GRAVITY
	if(invSphereMass!=Scalar(0))
		sphereVelocity+=dv;
	#endif
	sphereTimeStamp=Scalar(0);
	
	#if ACCUMULATE_PRESSURE
	pressureTime+=timeStep;
	#endif
	}

template <class ScalarParam,int dimensionParam>
inline
typename CollisionBox<ScalarParam,dimensionParam>::Scalar
CollisionBox<ScalarParam,dimensionParam>::calcAverageSpeed(
	const typename CollisionBox<ScalarParam,dimensionParam>::Box& queryBox) const
	{
	/* Find the extent of cells covered by the query box: */
	Index min,max;
	for(int i=0;i<dimension;++i)
		{
		min[i]=int(Math::floor(queryBox.min[i]/cellSize[i]));
		if(min[i]<0)
			min[i]=0;
		max[i]=int(Math::floor(queryBox.max[i]/cellSize[i]))+1;
		if(max[i]>numCells[i])
			max[i]=numCells[i];
		
		/* Account for ghost cells: */
		++min[i];
		++max[i];
		}
	
	/* Process all grid cells inside the range: */
	Scalar speedSum2(0);
	size_t numParticles=0;
	for(Index cellIndex=min;cellIndex[0]<max[0];cellIndex.preInc(min,max))
		{
		/* Process all particles inside the cell: */
		for(const Particle* pPtr=cells[cellIndex].particlesHead;pPtr!=0;pPtr=pPtr->cellSucc)
			{
			/* Check if the particle is inside the query box: */
			if(queryBox.contains(pPtr->position))
				{
				speedSum2+=pPtr->velocity.sqr();
				++numParticles;
				}
			}
		}
	
	/* Return the average speed: */
	return Math::sqrt(speedSum2/Scalar(numParticles));
	}


template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::saveState(
	IO::File& file) const
	{
	/* Write the collision box vector space dimension and scalar type for compatibility checks: */
	file.write<Misc::UInt32>(dimension);
	file.write<Misc::UInt32>(sizeof(Scalar));
	
	/* Write the collision box boundaries: */
	file.write(boundaries.min.getComponents(),dimension);
	file.write(boundaries.max.getComponents(),dimension);
	
	/* Write the cell size and number of cells: */
	file.write(cellSize.getComponents(),dimension);
	file.write(numCells,dimension);
	
	/* Write simulation parameters: */
	file.write(particleRadius);
	file.write(gravity.getComponents(),dimension);
	file.write(attenuation);
	
	/* Write the state of the spherical obstacle: */
	file.write(spherePosition.getComponents(),dimension);
	file.write(sphereVelocity.getComponents(),dimension);
	file.write(sphereRadius);
	file.write<Misc::UInt8>(sphereOpen?1:0);
	file.write(invSphereMass);
	
	/* Write the states of all particles: */
	file.write<Misc::UInt32>(numParticles);
	for(typename ParticleList::const_iterator pIt=particles.begin();pIt!=particles.end();++pIt)
		{
		file.write(pIt->getPosition().getComponents(),dimension);
		file.write(pIt->getVelocity().getComponents(),dimension);
		}
	}

#if ACCUMULATE_PRESSURE

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::accumulatePressure(
	unsigned int newNumPressureSlots)
	{
	/* Re-initialize the pressure accumulator: */
	numPressureSlots=newNumPressureSlots;
	pressureSlotSize=(boundaries.max[1]-boundaries.min[1])/Scalar(numPressureSlots);
	delete[] pressureSlots;
	pressureSlots=new Scalar[numPressureSlots*2];
	
	/* Restart pressure accumulation: */
	for(unsigned int i=0;i<numPressureSlots*2;++i)
		pressureSlots[i]=Scalar(0);
	pressureTime=Scalar(0);
	}

#endif

template <class ScalarParam,int dimensionParam>
inline
void
CollisionBox<ScalarParam,dimensionParam>::fire(
	const typename CollisionBox<ScalarParam,dimensionParam>::Point& firePos,
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar fireRadius,
	typename CollisionBox<ScalarParam,dimensionParam>::Scalar fireSpeed)
	{
	Scalar fireRadius2=Math::sqr(fireRadius);
	for(typename ParticleList::iterator pIt=particles.begin();pIt!=particles.end();++pIt)
		if(Geometry::sqrDist(pIt->position,firePos)<fireRadius2)
			pIt->velocity*=fireSpeed; // /pIt->velocity.mag();
	}
