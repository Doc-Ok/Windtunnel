/***********************************************************************
CollisionBoxTest - Test program for the collision box hard sphere
simulation data structure.
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

#define DRAW_PARTICLES_SPHERES 1
#define HAVE_SPHERERENDERER 1
#define DRAW_PARTICLES_SPRITES 0

#include <stdlib.h>
#include <string.h>
#include <stdexcept>
#include <iostream>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Math/Random.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/OrthogonalTransformation.h>
#include <GL/gl.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLMaterialTemplates.h>
#include <GL/GLObject.h>
#include <GL/GLContextData.h>
#include <GL/GLVertexArrayParts.h>
#include <GL/Extensions/GLARBVertexBufferObject.h>
#include <GL/Extensions/GLARBPointParameters.h>
#include <GL/Extensions/GLARBPointSprite.h>
#include <GL/Extensions/GLARBShaderObjects.h>
#include <GL/Extensions/GLARBFragmentShader.h>
#include <GL/Extensions/GLARBVertexShader.h>
#if HAVE_SPHERERENDERER
#include <GL/GLSphereRenderer.h>
#endif
#include <GL/GLColorMap.h>
#include <GL/GLModels.h>
#include <GL/GLGeometryWrappers.h>
#include <GL/GLGeometryVertex.h>
#include <GL/GLFrustum.h>
#include <GL/GLShader.h>
#include <Vrui/LocatorToolAdapter.h>
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>
#include <Vrui/VRWindow.h>

#include "CollisionBox.h"

class CollisionBoxTest:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */
	private:
	typedef CollisionBox<double,2> MyCollisionBox;
	typedef MyCollisionBox::Scalar Scalar;
	typedef MyCollisionBox::Point Point;
	typedef MyCollisionBox::Vector Vector;
	
	class SphereLocator:public Vrui::LocatorToolAdapter
		{
		/* Elements: */
		public:
		CollisionBoxTest* application; // Pointer to the application object
		bool active; // Flag if the button is currently pressed
		
		/* Constructors and destructors: */
		SphereLocator(Vrui::LocatorTool* sTool,CollisionBoxTest* sApplication);
		
		/* Methods: */
		virtual void motionCallback(Vrui::LocatorTool::MotionCallbackData* cbData);
		virtual void buttonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData);
		virtual void buttonReleaseCallback(Vrui::LocatorTool::ButtonReleaseCallbackData* cbData);
		};
	
	typedef GLGeometry::Vertex<void,0,GLubyte,4,void,GLfloat,MyCollisionBox::dimension> Vertex; // Type for vertex buffer elements
	
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint vertexBufferId; // ID of vertex buffer holding current and old particle positions
		unsigned int vertexBufferVersion; // Version number of vertex buffer
		GLuint boxListId; // ID of display list to draw the collision box
		GLuint ballListId; // ID of display list to render balls
		GLuint sphereListIdBase; // Base ID of display lists to render a spherical obstacle closed and open
		bool havePointSprites; // Flag whether local OpenGL supports point parameter and point sprite extensions
		bool haveShaders; // Flag whether local OpenGL supports GLSL shaders
		GLhandleARB vertexShaderObject,fragmentShaderObject,programObject; // Shader for proper point size attenuation
		GLint scaledParticleRadiusLocation; // Location of particle radius uniform variable in shader program
		GLint tex0Location; // Location of texture sample uniform variable in shader program
		GLfloat maxPointSize; // Maximum point size accepted by glPointSize() function
		GLuint particleTextureObjectId; // ID of the particle texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	private:
	Scalar particleRadius; // Model coordinate radius of particles in the collision box
	Scalar attenuation; // Particle velocity attenuation
	MyCollisionBox* collisionBox; // The collision box data structure
	SphereLocator* sphereLocator; // Pointer to a locator adapter
	Point spherePosition; // New position for the collision sphere
	bool useSphere; // Use a collision sphere
	bool sphereOpen; // Flag if the sphere is currently open
	double lastApplicationTime; // Application time at last invocation of frame method
	unsigned numSlots; // Number of particle ring buffer slots
	unsigned int currentSlotIndex; // Index of the particle ring buffer slot containing most recent particle states
	unsigned int ringBufferVersion; // Version number of particle ring buffer
	bool useColorMap; // Flag to color particles by their speed
	bool drawTrails; // Flag to draw particle trails
	bool fire; // Flag if fire is active
	#if HAVE_SPHERERENDERER
	GLSphereRenderer sphereRenderer; // Helper object to render particles as spheres
	#endif
	GLColorMap* colorMap; // Map to color particles by their kinetic energy
	unsigned int screenShotIndex;
	
	/* Private methods: */
	void resetTempScale(void); // Resets the temperature color map scale
	
	/* Constructors and destructors: */
	public:
	CollisionBoxTest(int& argc,char**& argv); // Initializes the Vrui toolkit and the application
	virtual ~CollisionBoxTest(void);
	
	/* Methods from class Vrui::Application: */
	virtual void toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData);
	virtual void toolDestructionCallback(Vrui::ToolManager::ToolDestructionCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	virtual void resetNavigation(void);
	virtual void eventCallback(EventID eventId,Vrui::InputDevice::ButtonCallbackData* cbData);
	
	/* Methods from class GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	};

/************************************************
Methods of class CollisionBoxTest::SphereLocator:
************************************************/

CollisionBoxTest::SphereLocator::SphereLocator(Vrui::LocatorTool* sLocatorTool,CollisionBoxTest* sApplication)
	:LocatorToolAdapter(sLocatorTool),
	 application(sApplication)
	{
	}

void CollisionBoxTest::SphereLocator::motionCallback(Vrui::LocatorTool::MotionCallbackData* cbData)
	{
	if(active)
		{
		/* Get the current position: */
		Vrui::Point v=cbData->currentTransformation.getOrigin();
		
		/* Set the new sphere position: */
		for(int i=0;i<MyCollisionBox::dimension;++i)
			application->spherePosition[i]=Scalar(v[i]);
		}
	}

void CollisionBoxTest::SphereLocator::buttonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData)
	{
	/* Start dragging: */
	active=true;
	}

void CollisionBoxTest::SphereLocator::buttonReleaseCallback(Vrui::LocatorTool::ButtonReleaseCallbackData* cbData)
	{
	/* Stop dragging: */
	active=false;
	}

/*******************************************
Methods of class CollisionBoxTest::DataItem:
*******************************************/

CollisionBoxTest::DataItem::DataItem(void)
	:vertexBufferId(0),vertexBufferVersion(0),
	 boxListId(glGenLists(1)),ballListId(glGenLists(1)),sphereListIdBase(glGenLists(2)),
	 havePointSprites(GLARBPointParameters::isSupported()&&GLARBPointSprite::isSupported()),
	 haveShaders(GLARBShaderObjects::isSupported()&&GLARBVertexShader::isSupported()&&GLARBFragmentShader::isSupported()),
	 vertexShaderObject(0),fragmentShaderObject(0),programObject(0),
	 particleTextureObjectId(0)
	{
	GLARBVertexBufferObject::initExtension();
	glGenBuffersARB(1,&vertexBufferId);
	
	if(havePointSprites)
		{
		/* Initialize the OpenGL extensions: */
		GLARBPointParameters::initExtension();
		GLARBPointSprite::initExtension();
		
		/* Create the particle texture object: */
		glGenTextures(1,&particleTextureObjectId);
		}
	
	if(haveShaders)
		{
		/* Initialize the OpenGL extensions: */
		GLARBShaderObjects::initExtension();
		GLARBVertexShader::initExtension();
		GLARBFragmentShader::initExtension();
		}
	}

CollisionBoxTest::DataItem::~DataItem(void)
	{
	glDeleteBuffersARB(1,&vertexBufferId);
	
	glDeleteLists(boxListId,1);
	glDeleteLists(ballListId,1);
	glDeleteLists(sphereListIdBase,2);
	if(havePointSprites)
		glDeleteTextures(1,&particleTextureObjectId);
	
	if(haveShaders)
		{
		glDeleteObjectARB(programObject);
		glDeleteObjectARB(vertexShaderObject);
		glDeleteObjectARB(fragmentShaderObject);
		}
	}

/*********************************
Methods of class CollisionBoxTest:
*********************************/

namespace {

void parse(char**& argPtr,char** argEnd,int& value)
	{
	/* Check if there is another argument: */
	++argPtr;
	if(argPtr!=argEnd)
		{
		/* Parse the value: */
		value=atoi(*argPtr);
		}
	else
		std::cerr<<"Ignoring dangling command line option "<<argPtr[-1]<<std::endl;
	}

void parse(char**& argPtr,char** argEnd,unsigned int& value)
	{
	/* Check if there is another argument: */
	++argPtr;
	if(argPtr!=argEnd)
		{
		/* Parse the value: */
		value=atoi(*argPtr);
		}
	else
		std::cerr<<"Ignoring dangling command line option "<<argPtr[-1]<<std::endl;
	}

void parse(char**& argPtr,char** argEnd,float& value)
	{
	/* Check if there is another argument: */
	++argPtr;
	if(argPtr!=argEnd)
		{
		/* Parse the value: */
		value=float(atof(*argPtr));
		}
	else
		std::cerr<<"Ignoring dangling command line option "<<argPtr[-1]<<std::endl;
	}

void parse(char**& argPtr,char** argEnd,double& value)
	{
	/* Check if there is another argument: */
	++argPtr;
	if(argPtr!=argEnd)
		{
		/* Parse the value: */
		value=atof(*argPtr);
		}
	else
		std::cerr<<"Ignoring dangling command line option "<<argPtr[-1]<<std::endl;
	}

void parse(char**& argPtr,char** argEnd,int numValues,float* values)
	{
	/* Check if there are enough additional arguments: */
	if(argEnd-argPtr>numValues)
		{
		for(int index=0;index<numValues;++index)
			{
			++argPtr;
			values[index]=float(atof(*argPtr));
			}
		}
	else
		{
		std::cerr<<"Ignoring dangling command line option "<<*argPtr<<std::endl;
		argPtr=argEnd;
		}
	}

void parse(char**& argPtr,char** argEnd,int numValues,double* values)
	{
	/* Check if there are enough additional arguments: */
	if(argEnd-argPtr>numValues)
		{
		for(int index=0;index<numValues;++index)
			{
			++argPtr;
			values[index]=atof(*argPtr);
			}
		}
	else
		{
		std::cerr<<"Ignoring dangling command line option "<<*argPtr<<std::endl;
		argPtr=argEnd;
		}
	}

}

void CollisionBoxTest::resetTempScale(void)
	{
	/* Calculate the average squared particle velocity: */
	Scalar vel2Sum(0);
	for(MyCollisionBox::ParticleList::const_iterator pIt=collisionBox->getParticles().begin();pIt!=collisionBox->getParticles().end();++pIt)
		vel2Sum+=pIt->getVelocity().sqr();
	vel2Sum/=Scalar(collisionBox->getParticles().size());
	
	/* Create a new color map: */
	static const GLColorMap::Color colors[3]=
		{
		// GLColorMap::Color(0.0,0.0,1.0),GLColorMap::Color(0.5,0.0,0.5),GLColorMap::Color(1.0,0.0,0.0)
		GLColorMap::Color(0.0,0.0,1.0),GLColorMap::Color(1.0,1.0,1.0),GLColorMap::Color(1.0,0.0,0.0)
		};
	GLdouble keys[3];
	keys[0]=vel2Sum/Scalar(2);
	keys[1]=vel2Sum;
	keys[2]=vel2Sum*Scalar(2);
	delete colorMap;
	colorMap=new GLColorMap(3,colors,keys);
	}

CollisionBoxTest::CollisionBoxTest(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 particleRadius(1.0),attenuation(0.95),
	 collisionBox(0),
	 sphereLocator(0),
	 useSphere(true),sphereOpen(false),
	 lastApplicationTime(Vrui::getApplicationTime()),
	 numSlots(1),currentSlotIndex(numSlots-1),ringBufferVersion(0),
	 useColorMap(true),drawTrails(false),
	 fire(false),
	 colorMap(0),
	 screenShotIndex(0)
	{
	/* Parse the command line: */
	const char* stateFileName=0;
	Vector boxSize;
	for(int i=0;i<MyCollisionBox::dimension;++i)
		boxSize[i]=Scalar(200);
	int numParticles=1000;
	Scalar speed(4);
	Vector gravity=Vector::zero;
	Scalar sphereRadius(25);
	Scalar invSphereMass(0);
	bool inSphere=false;
	char** argEnd=argv+argc;
	for(char** argPtr=argv+1;argPtr!=argEnd;)
		{
		if((*argPtr)[0]=='-')
			{
			if(strcasecmp(*argPtr+1,"boxSize")==0||strcasecmp(*argPtr+1,"bs")==0)
				parse(argPtr,argEnd,MyCollisionBox::dimension,boxSize.getComponents());
			else if(strcasecmp(*argPtr+1,"numParticles")==0||strcasecmp(*argPtr+1,"np")==0)
				parse(argPtr,argEnd,numParticles);
			else if(strcasecmp(*argPtr+1,"particleRadius")==0||strcasecmp(*argPtr+1,"pr")==0)
				parse(argPtr,argEnd,particleRadius);
			else if(strcasecmp(*argPtr+1,"gravity")==0||strcasecmp(*argPtr+1,"g")==0)
				parse(argPtr,argEnd,MyCollisionBox::dimension,gravity.getComponents());
			else if(strcasecmp(*argPtr+1,"attenuation")==0||strcasecmp(*argPtr+1,"att")==0)
				parse(argPtr,argEnd,attenuation);
			else if(strcasecmp(*argPtr+1,"speed")==0||strcasecmp(*argPtr+1,"sp")==0)
				parse(argPtr,argEnd,speed);
			else if(strcasecmp(*argPtr+1,"sphereRadius")==0||strcasecmp(*argPtr+1,"sr")==0)
				parse(argPtr,argEnd,sphereRadius);
			else if(strcasecmp(*argPtr+1,"invSphereMass")==0||strcasecmp(*argPtr+1,"ism")==0)
				parse(argPtr,argEnd,invSphereMass);
			else if(strcasecmp(*argPtr+1,"trailLength")==0||strcasecmp(*argPtr+1,"tl")==0)
				parse(argPtr,argEnd,numSlots);
			else if(strcasecmp(*argPtr+1,"inSphere")==0||strcasecmp(*argPtr+1,"is")==0)
				inSphere=true;
			else if(strcasecmp(*argPtr+1,"noSphere")==0||strcasecmp(*argPtr+1,"ns")==0)
				useSphere=false;
			else if(strcasecmp(*argPtr+1,"noColorMap")==0||strcasecmp(*argPtr+1,"ncm")==0)
				useColorMap=false;
			}
		else if(stateFileName==0)
			stateFileName=*argPtr;
		else
			std::cerr<<"Ignoring command line argument "<<*argPtr<<std::endl;
		
		if(argPtr!=argEnd)
			++argPtr;
		}
	
	if(stateFileName!=0)
		{
		/* Load a state file: */
		IO::FilePtr file=IO::openFile(stateFileName);
		file->setEndianness(Misc::LittleEndian);
		collisionBox=new MyCollisionBox(*file);
		file=0;
		
		/* EVIL HACK: Override attenuation for circular simulations: */
		// collisionBox->setAttenuation(0.99843);
		
		/* Retrieve collision box parameters: */
		particleRadius=collisionBox->getParticleRadius();
		attenuation=collisionBox->getAttenuation();
		
		/* Override spherical obstacle mass: */
		if(invSphereMass!=Scalar(0))
			collisionBox->setInverseSphereMass(invSphereMass);
		}
	else
		{
		/* Create a collision box: */
		Point min,max;
		for(int i=0;i<MyCollisionBox::dimension;++i)
			{
			min[i]=Scalar(0);
			max[i]=boxSize[i];
			}
		
		collisionBox=new MyCollisionBox(MyCollisionBox::Box(min,max),particleRadius,sphereRadius);
		collisionBox->setAttenuation(attenuation);
		collisionBox->setGravity(gravity);
		if(useSphere)
			{
			/* Put the sphere into the box and set its mass: */
			collisionBox->setSphere(Geometry::mid(min,max));
			collisionBox->setInverseSphereMass(invSphereMass);
			}
		
		/* Create a few particles: */
		const MyCollisionBox::Box& boundaries=collisionBox->getBoundaries();
		int particleIndex;
		for(particleIndex=0;particleIndex<numParticles;++particleIndex)
			{
			const int maxNumTries=200;
			int tries;
			for(tries=0;tries<maxNumTries;++tries)
				{
				Point p;
				Vector v;
				if(inSphere)
					{
					while(true)
						{
						Scalar r2(0);
						for(int j=0;j<MyCollisionBox::dimension;++j)
							{
							p[j]=Math::randUniformCC(collisionBox->getSphere()[j]-sphereRadius+particleRadius,collisionBox->getSphere()[j]+sphereRadius-particleRadius);
							r2+=Math::sqr(p[j]-collisionBox->getSphere()[j]);
							}
						if(r2<Math::sqr(sphereRadius-particleRadius))
							break;
						}
					for(int j=0;j<MyCollisionBox::dimension;++j)
						v[j]=Scalar(Math::randUniformCC(-speed,speed));
					}
				else
					{
					for(int j=0;j<MyCollisionBox::dimension;++j)
						{
						p[j]=Math::randUniformCC(boundaries.min[j]+particleRadius,boundaries.max[j]-particleRadius);
						#if 0
						if(j==1)
							p[j]=Math::randUniformCC(boundaries.min[j]+particleRadius,boundaries.min[j]*0.667+boundaries.max[j]*0.333);
						#endif
						v[j]=Scalar(Math::randUniformCC(-speed,speed));
						}
					}
				
				/* Try adding the new particle: */
				if(collisionBox->addParticle(p,v,!inSphere))
					break;
				}
			
			if(tries==maxNumTries) // Could not add particles after N tries; assume box is full
				break;
			}
		
		if(particleIndex<numParticles)
			{
			/* Print a warning message: */
			std::cerr<<"Could only add "<<particleIndex<<" particles to collision box"<<std::endl;
			}
		}
	
	/* Retrieve the position of the spherical obstacle: */
	spherePosition=collisionBox->getSphere();
	
	#if HAVE_SPHERERENDERER
	/* Initialize the sphere renderer: */
	sphereRenderer.setFixedRadius(particleRadius);
	sphereRenderer.setColorMaterial(true);
	#endif
	
	#if ACCUMULATE_PRESSURE
	/* Start pressure accumulation: */
	collisionBox->accumulatePressure(50);
	#endif
	
	/* Create a color map: */
	resetTempScale();
	
	/* Create an event tool class to open the sphere: */
	addEventTool("Open/Close Sphere",0,0);
	addEventTool("Cool",0,1);
	addEventTool("Heat",0,2);
	addEventTool("Save",0,3);
	addEventTool("Draw Trails",0,4);
	addEventTool("Calc Pressure",0,5);
	addEventTool("Reset Temp",0,6);
	addEventTool("Kaboom",0,7);
	addEventTool("Toggle Color Map",0,8);
	addEventTool("Sphere Light",0,9);
	addEventTool("Sphere Medium",0,10);
	addEventTool("Sphere Heavy",0,11);
	}

CollisionBoxTest::~CollisionBoxTest(void)
	{
	delete collisionBox;
	delete colorMap;
	}

void CollisionBoxTest::toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData)
	{
	/* Check if the new tool is a locator tool: */
	Vrui::LocatorTool* locatorTool=dynamic_cast<Vrui::LocatorTool*>(cbData->tool);
	if(locatorTool!=0&&sphereLocator==0)
		{
		/* Create a sphere locator object and associate it with the new tool: */
		sphereLocator=new SphereLocator(locatorTool,this);
		}
	}

void CollisionBoxTest::toolDestructionCallback(Vrui::ToolManager::ToolDestructionCallbackData* cbData)
	{
	/* Check if the to-be-destroyed tool is a locator tool: */
	Vrui::LocatorTool* locatorTool=dynamic_cast<Vrui::LocatorTool*>(cbData->tool);
	if(locatorTool!=0&&sphereLocator!=0&&locatorTool==sphereLocator->getTool())
		{
		delete sphereLocator;
		sphereLocator=0;
		}
	}

void CollisionBoxTest::frame(void)
	{
	/* Simulate by the current frame time: */
	double newApplicationTime=Vrui::getApplicationTime();
	double timeStep=newApplicationTime-lastApplicationTime;
	if(timeStep>1.0/30.0)
		{
		/* Go into "catch-up" mode: */
		timeStep=1.0/30.0;
		// collisionBox->setAttenuation(Scalar(0.5));
		}
	else
		{
		// collisionBox->setAttenuation(Scalar(attenuation));
		}
	
	/* Use a fixed time step to record movies: */
	timeStep=1.0/30.0;
	
	/* Let a locator tool drag the spherical obstacle: */
	// collisionBox->moveSphere(spherePosition,0.1);
	
	/* Update the simulation state: */
	collisionBox->simulate(timeStep);
	
	lastApplicationTime=newApplicationTime;
	
	#if 0
	/* Calculate total kinetic energy: */
	Scalar energy(0);
	for(MyCollisionBox::ParticleList::const_iterator pIt=collisionBox->getParticles().begin();pIt!=collisionBox->getParticles().end();++pIt)
		{
		energy+=Scalar(0.5)*pIt->getVelocity().sqr();
		energy+=Scalar(9.81)*Geometry::dist(pIt->getPosition(),collisionBox->getSphere());
		// energy+=Scalar(9.81)*pIt->getPosition()[1];
		}
	std::cout<<"\r"<<energy<<"    "<<std::flush;
	#endif
	
	if(fire)
		{
		/* Set a fire: */
		Point firePoint=collisionBox->getSphere();
		firePoint[1]+=collisionBox->getSphereRadius()+Scalar(2.5);
		collisionBox->fire(firePoint,Scalar(2.5),Math::pow(Scalar(1000),timeStep));
		}
	
	/* Update particle ring buffer state: */
	currentSlotIndex=(currentSlotIndex+1)%numSlots;
	++ringBufferVersion;
	
	/* Schedule another frame: */
	Vrui::scheduleUpdate(Vrui::getNextAnimationTime());
	
	#if 0
	/* Request a screenshot: */
	std::cout<<"\r"<<screenShotIndex<<std::flush;
	char ssfnb[1024];
	snprintf(ssfnb,sizeof(ssfnb),"/work/okreylos/MovieData/Gas/SphericalGasFrames/Frame%06u.png",screenShotIndex);
	Vrui::getWindow(0)->requestScreenshot(ssfnb);
	++screenShotIndex;
	#endif
	}

void CollisionBoxTest::display(GLContextData& contextData) const
	{
	/* Set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	
	/* Get the OpenGL-dependent application data from the GLContextData object: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/*********************************************************************
	Render non-particle states:
	*********************************************************************/
	
	/* Draw the collision box: */
	glCallList(dataItem->boxListId);
	
	if(useSphere)
		{
		/* Draw the spherical obstacle: */
		glPushMatrix();
		glTranslate(collisionBox->getSphere()-Point::origin);
		glCallList(dataItem->sphereListIdBase+(sphereOpen?1:0));
		glPopMatrix();
		}
	
	#if ACCUMULATE_PRESSURE
	
	if(MyCollisionBox::dimension==2)
		{
		/* Find the maximum pressure for scaling: */
		int numPs=collisionBox->getNumPressureSlots();
		const Scalar* pss=collisionBox->getPressureSlots();
		Scalar pt=collisionBox->getPressureTime();
		Scalar maxP(0);
		for(int psi=1;psi<numPs-1;++psi)
			for(int i=0;i<2;++i)
				{
				Scalar p=pss[psi*2+i]/pt;
				if(maxP<p)
					maxP=p;
				}
		
		/* Visualize boundary pressure: */
		glDisable(GL_LIGHTING);
		glBegin(GL_QUADS);
		glColor3f(1.0f,0.0f,0.0f);
		Scalar pScale=Scalar(0.25)*(boundaries.max[0]-boundaries.min[0])/(pt*maxP);
		Scalar xl=boundaries.min[0]-(boundaries.max[0]-boundaries.min[0])*Scalar(0.01);
		Scalar xr=boundaries.max[0]+(boundaries.max[0]-boundaries.min[0])*Scalar(0.01);
		for(int psi=1;psi<numPs-1;++psi)
			{
			Scalar y0=(Scalar(psi)+Scalar(0.25))*(boundaries.max[1]-boundaries.min[1])/Scalar(numPs)+boundaries.min[1];
			Scalar y1=(Scalar(psi)+Scalar(0.75))*(boundaries.max[1]-boundaries.min[1])/Scalar(numPs)+boundaries.min[1];
			
			/* Visualize the left-wall pressure: */
			Scalar pl=pss[psi*2+0]*pScale;
			glVertex2d(xl-pl,y0);
			glVertex2d(xl,y0);
			glVertex2d(xl,y1);
			glVertex2d(xl-pl,y1);
			
			/* Visualize the right-wall pressure: */
			Scalar pr=pss[psi*2+1]*pScale;
			glVertex2d(xr,y0);
			glVertex2d(xr+pr,y0);
			glVertex2d(xr+pr,y1);
			glVertex2d(xr,y1);
			}
		glEnd();
		}
	
	#endif
	
	/* Bind the particle ring buffer: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->vertexBufferId);
	
	/* Check if the particle ring buffer is out of date: */
	unsigned int numParticles=collisionBox->getParticles().size();
	if(dataItem->vertexBufferVersion!=ringBufferVersion)
		{
		/* Upload current particle states into the next particle ring buffer slot: */
		Vertex* vPtr=static_cast<Vertex*>(glMapBufferARB(GL_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
		vPtr+=currentSlotIndex*numParticles;
		if(useColorMap)
			{
			/* Color particles by temperature: */
			for(MyCollisionBox::ParticleList::const_iterator pIt=collisionBox->getParticles().begin();pIt!=collisionBox->getParticles().end();++pIt,++vPtr)
				{
				vPtr->color=(*colorMap)(pIt->getVelocity().sqr());
				vPtr->position=Vertex::Position(pIt->getPosition());
				}
			}
		else
			{
			/* Draw all particles the same color: */
			for(MyCollisionBox::ParticleList::const_iterator pIt=collisionBox->getParticles().begin();pIt!=collisionBox->getParticles().end();++pIt,++vPtr)
				{
				vPtr->color=Vertex::Color(128,128,128);
				vPtr->position=Vertex::Position(pIt->getPosition());
				}
			}
		glUnmapBufferARB(GL_ARRAY_BUFFER_ARB);
		
		/* Mark the particle ring buffer as current: */
		dataItem->vertexBufferVersion=ringBufferVersion;
		}
	
	/*********************************************************************
	Render all particles:
	*********************************************************************/
	
	/* Draw all particles: */
	GLVertexArrayParts::enable(Vertex::getPartsMask());
	glVertexPointer(static_cast<const Vertex*>(0));
	
	#if DRAW_PARTICLES_SPHERES
	
	/* Draw current particle states: */
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.5f,0.5f,0.5f));
	glMaterialSpecular(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,1.0f,1.0f));
	glMaterialShininess(GLMaterialEnums::FRONT,64.0f);
	
	#if HAVE_SPHERERENDERER
	
	/* Draw the particles using impostor spheres: */
	sphereRenderer.enable(GLfloat(Vrui::getNavigationTransformation().getScaling()),contextData);
	glDrawArrays(GL_POINTS,currentSlotIndex*numParticles,numParticles);
	sphereRenderer.disable(contextData);
	
	#else
	
	/* Draw the particles using geometry: */
	
	glPushMatrix();
	Point currentPos=Point::origin;
	glColor3f(0.5f,0.5f,0.5f);
	for(MyCollisionBox::ParticleList::const_iterator pIt=collisionBox->getParticles().begin();pIt!=collisionBox->getParticles().end();++pIt)
		{
		glTranslate(pIt->getPosition()-currentPos);
		glCallList(dataItem->ballListId);
		currentPos=pIt->getPosition();
		}
	glPopMatrix();
	
	#endif
	
	if(drawTrails)
		{
		/* Draw particle trails: */
		glDisable(GL_LIGHTING);
		glPointSize(1.0f);
		glDrawArrays(GL_POINTS,0,numSlots*numParticles);
		}
	
	#endif
	
	#if DRAW_PARTICLES_SPRITES
	
	/* Enable additive blending: */
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE,GL_ONE);
	
	if(dataItem->havePointSprites)
		{
		/* Enable point sprites: */
		glEnable(GL_POINT_SPRITE_ARB);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D,dataItem->particleTextureObjectId);
		glTexEnvi(GL_POINT_SPRITE_ARB,GL_COORD_REPLACE_ARB,GL_TRUE);
		glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
		
		/* Query the OpenGL viewing frustum: */
		GLFrustum<float> frustum;
		frustum.setFromGL();
		
		if(dataItem->haveShaders)
			{
			/* Calculate the scaled point size for this frustum: */
			GLfloat scaledParticleRadius=frustum.getPixelSize()*particleRadius/frustum.getEyeScreenDistance();
			scaledParticleRadius*=GLfloat(Vrui::getNavigationTransformation().getScaling());
			
			/* Set up the sprite shader: */
			glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_ARB);
			glUseProgramObjectARB(dataItem->programObject);
			glUniform1fARB(dataItem->scaledParticleRadiusLocation,scaledParticleRadius);
			glUniform1iARB(dataItem->tex0Location,0);
			
			/* Draw all particles: */
			glDrawArrays(GL_POINTS,currentSlotIndex*numParticles,numParticles);
			
			/* Disable the sprite shader: */
			glUseProgramObjectARB(0);
			glDisable(GL_VERTEX_PROGRAM_POINT_SIZE_ARB);
			}
		else
			{
			/* Calculate the nominal pixel size for particles: */
			float pointSizeCounter=frustum.getPixelSize()*2.0f*particleRadius;
			float pointSizeDenominator=frustum.getEyeScreenDistance();
			pointSizeDenominator/=Vrui::getNavigationTransformation().getScaling();
			
			/* Query the maximum point size: */
			GLfloat pointSizeRange[2];
			glGetFloatv(GL_SMOOTH_POINT_SIZE_RANGE,pointSizeRange);
			
			/* Enable point parameters: */
			glPointSize(pointSizeRange[1]);
			GLfloat attenuation[3]={0.0f,0.0f,0.0f};
			attenuation[2]=Math::sqr(pointSizeRange[1]*pointSizeDenominator/pointSizeCounter);
			glPointParameterfvARB(GL_POINT_DISTANCE_ATTENUATION_ARB,attenuation);
			
			/* Draw all particles: */
			glDrawArrays(GL_POINTS,currentSlotIndex*numParticles,numParticles);
			}
		
		/* Disable point sprites: */
		glBindTexture(GL_TEXTURE_2D,0);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_POINT_SPRITE_ARB);
		}
	
	/* Disable additive blending: */
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	
	#endif
	
	GLVertexArrayParts::disable(Vertex::getPartsMask());
	
	/* Protect the particle ring buffer: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	
	/* Restore OpenGL state: */
	glPopAttrib();
	}

void CollisionBoxTest::resetNavigation(void)
	{
	/* Calculate the center and size of the collision box: */
	const MyCollisionBox::Box& boundaries=collisionBox->getBoundaries();
	Vrui::Point center=Vrui::Point::origin;
	Vrui::Scalar radius(0);
	for(int i=0;i<MyCollisionBox::dimension;++i)
		{
		center[i]=Math::div2(boundaries.min[i]+boundaries.max[i]);
		radius+=Math::sqr(boundaries.getSize(i));
		}
	radius=Math::div2(Math::sqrt(radius));
	
	/* Reset the Vrui navigation transformation: */
	Vrui::setNavigationTransformation(center,radius,Vrui::Vector(0,1,0));
	}

void CollisionBoxTest::eventCallback(Vrui::Application::EventID eventId,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		{
		switch(eventId)
			{
			case 0:
				/* Open or close the spherical obstacle: */
				sphereOpen=!sphereOpen;
				collisionBox->setSphereOpen(sphereOpen);
				break;
			
			case 1:
				/* Set positive attenuation to remove energy from the simulation: */
				collisionBox->setAttenuation(1.0/1.5); // (1.0/1.1);
				break;
			
			case 2:
				/* Set negative attenuation to add energy to the simulation: */
				collisionBox->setAttenuation(1.1);
				break;
			
			case 3:
				{
				/* Save the current simulation state to a file: */
				IO::FilePtr file=IO::openFile("SavedCollisionBoxState.dat",IO::File::WriteOnly);
				file->setEndianness(Misc::LittleEndian);
				collisionBox->saveState(*file);
				file=0;
				
				break;
				}
			
			case 4:
				/* Turn particle trails on or off: */
				drawTrails=!drawTrails;
				break;
			
			case 5:
				// collisionBox->accumulatePressure(50);
				break;
			
			case 6:
				/* Re-align the temperature color scale: */
				resetTempScale();
				break;
			
			case 7:
				/* Cause a gigantic explosion: */
				fire=true;
				break;
			
			case 8:
				/* Turn temperature color mapping on or off: */
				useColorMap=!useColorMap;
				break;
			
			case 9:
				/* Set the spherical obstacle's mass to "light": */
				collisionBox->setInverseSphereMass(0.006);
				break;
			
			case 10:
				/* Set the spherical obstacle's mass to "medium": */
				collisionBox->setInverseSphereMass(0.0006);
				break;
			
			case 11:
				/* Set the spherical obstacle's mass to "heavy": */
				collisionBox->setInverseSphereMass(0.00006);
				break;
			}
		}
	else
		{
		switch(eventId)
			{
			case 1:
			case 2:
				/* Set attenuation back to the requested value: */
				collisionBox->setAttenuation(attenuation);
				break;
			
			case 5:
				{
				/* Print the current pressure bars: */
				#if ACCUMULATE_PRESSURE
				const Scalar* ps=collisionBox->getPressureSlots();
				Scalar pt=collisionBox->getPressureTime();
				
				for(unsigned int i=0;i<50;++i)
					std::cout<<i<<','<<ps[i]/pt<<std::endl;
				#endif
				break;
				}
			
			case 7:
				/* Turn off the gigantic explosion: */
				fire=false;
				break;
			}
		}
	}

void CollisionBoxTest::initContext(GLContextData& contextData) const
	{
	/* Create context data item and store it in the GLContextData object: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	
	/* Create a display list to draw the collision box: */
	glNewList(dataItem->boxListId,GL_COMPILE);
	glDisable(GL_LIGHTING);
	glLineWidth(1.0f);
	glColor(Vrui::getForegroundColor());
	const MyCollisionBox::Box& boundaries=collisionBox->getBoundaries();
	if(MyCollisionBox::dimension==2)
		{
		/* Draw a 2D collision box: */
		glBegin(GL_LINE_LOOP);
		glVertex(boundaries.getVertex(0));
		glVertex(boundaries.getVertex(1));
		glVertex(boundaries.getVertex(3));
		glVertex(boundaries.getVertex(2));
		glEnd();
		}
	else
		{
		/* Draw a 3D collision box: */
		glBegin(GL_LINE_STRIP);
		glVertex(boundaries.getVertex(0));
		glVertex(boundaries.getVertex(1));
		glVertex(boundaries.getVertex(3));
		glVertex(boundaries.getVertex(2));
		glVertex(boundaries.getVertex(0));
		glVertex(boundaries.getVertex(4));
		glVertex(boundaries.getVertex(5));
		glVertex(boundaries.getVertex(7));
		glVertex(boundaries.getVertex(6));
		glVertex(boundaries.getVertex(4));
		glEnd();
		glBegin(GL_LINES);
		glVertex(boundaries.getVertex(1));
		glVertex(boundaries.getVertex(5));
		glVertex(boundaries.getVertex(3));
		glVertex(boundaries.getVertex(7));
		glVertex(boundaries.getVertex(2));
		glVertex(boundaries.getVertex(6));
		glEnd();
		}
	glEndList();
	
	/* Create a display list to draw a closed and open spherical obstacle: */
	if(MyCollisionBox::dimension==2)
		{
		GLfloat sr=GLfloat(collisionBox->getSphereRadius());
		
		/* Draw a closed sphere: */
		glNewList(dataItem->sphereListIdBase+0,GL_COMPILE);
		glDisable(GL_LIGHTING);
		glLineWidth(1.0f);
		glColor3f(1.0f,0.0f,0.0f);
		glBegin(GL_LINE_LOOP);
		for(int i=0;i<256;++i)
			{
			float angle=2.0f*Math::Constants<float>::pi*float(i)/float(256);
			glVertex2f(Math::cos(angle)*sr,Math::sin(angle)*sr);
			}
		glEnd();
		glEndList();
		
		/* Draw an open sphere: */
		glNewList(dataItem->sphereListIdBase+1,GL_COMPILE);
		glDisable(GL_LIGHTING);
		glLineWidth(1.0f);
		glColor3f(1.0f,0.0f,0.0f);
		Scalar alpha=Math::asin(Scalar(0.075));
		glBegin(GL_LINE_STRIP);
		for(int i=0;i<=256;++i)
			{
			float angle=(2.0f*Math::Constants<float>::pi-alpha*2.0f)*float(i)/float(256)+alpha;
			glVertex2f(Math::sin(angle)*sr,-Math::cos(angle)*sr);
			}
		glEnd();
		glEndList();
		}
	else
		{
		/* Draw a closed sphere: */
		glNewList(dataItem->sphereListIdBase+0,GL_COMPILE);
		glEnable(GL_LIGHTING);
		glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,0.0f,0.0f));
		glMaterialSpecular(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,1.0f,1.0f));
		glMaterialShininess(GLMaterialEnums::FRONT,64.0f);
		glDrawSphereIcosahedron(collisionBox->getSphereRadius(),12);
		glEndList();
		
		/* Draw an open sphere (not really bothering, though): */
		glNewList(dataItem->sphereListIdBase+1,GL_COMPILE);
		glEnable(GL_LIGHTING);
		glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,0.0f,0.0f));
		glMaterialSpecular(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,1.0f,1.0f));
		glMaterialShininess(GLMaterialEnums::FRONT,64.0f);
		glDrawSphereIcosahedron(collisionBox->getSphereRadius(),12);
		glEndList();
		}
	
	/* Create a display list to draw a little ball for particles: */
	glNewList(dataItem->ballListId,GL_COMPILE);
	if(MyCollisionBox::dimension==2)
		{
		glBegin(GL_POLYGON);
		for(int i=0;i<32;++i)
			{
			float angle=2.0f*Math::Constants<float>::pi*float(i)/float(32);
			glVertex2f(Math::cos(angle)*particleRadius,Math::sin(angle)*particleRadius);
			}
		glEnd();
		}
	else
		glDrawSphereIcosahedron(particleRadius,3);
	glEndList();
	
	if(dataItem->havePointSprites)
		{
		/* Create the particle texture: */
		GLfloat texImage[32][32][3];
		for(int y=0;y<32;++y)
			for(int x=0;x<32;++x)
				{
				float r2=Math::sqr((float(x)-15.5f)/15.5f)+Math::sqr((float(y)-15.5f)/15.5f);
				if(r2<1.0f)
					{
					float l=Math::exp(-r2*2.0f)-Math::exp(-2.0f);
					texImage[y][x][0]=l;
					texImage[y][x][1]=l;
					if(l<0.5f)
						texImage[y][x][2]=l*2.0f;
					else
						texImage[y][x][2]=1.0f;
					}
				else
					for(int i=0;i<3;++i)
						texImage[y][x][i]=0.0f;
				}
		
		/* Upload particle texture: */
		glBindTexture(GL_TEXTURE_2D,dataItem->particleTextureObjectId);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,32,32,0,GL_RGB,GL_FLOAT,&texImage[0][0][0]);
		glBindTexture(GL_TEXTURE_2D,0);
		}
	
	if(dataItem->haveShaders)
		{
		static const char* vertexProgram="\
			uniform float scaledParticleRadius; \
			void main() \
				{ \
				vec4 vertexEye; \
				 \
				/* Transform the vertex to eye coordinates: */ \
				vertexEye=gl_ModelViewMatrix*gl_Vertex; \
				 \
				/* Calculate point size based on vertex' eye distance along z direction: */ \
				gl_PointSize=scaledParticleRadius*2.0*vertexEye.w/vertexEye.z; \
				 \
				/* Use standard vertex position for fragment generation: */ \
				gl_FrontColor=vec4(1.0,1.0,1.0,1.0); \
				gl_Position=ftransform(); \
				}";
		static const char* fragmentProgram="\
			uniform sampler2D tex0; \
			 \
			void main() \
				{ \
				gl_FragColor=texture2D(tex0,gl_TexCoord[0].xy); \
				}";
		
		/* Compile and link the point size computation shader program: */
		dataItem->vertexShaderObject=glCompileVertexShaderFromString(vertexProgram);
		dataItem->fragmentShaderObject=glCompileFragmentShaderFromString(fragmentProgram);
		dataItem->programObject=glLinkShader(dataItem->vertexShaderObject,dataItem->fragmentShaderObject);
		dataItem->scaledParticleRadiusLocation=glGetUniformLocationARB(dataItem->programObject,"scaledParticleRadius");
		dataItem->tex0Location=glGetUniformLocationARB(dataItem->programObject,"tex0");
		}
	
	/* Initialize the particle state ring buffer: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->vertexBufferId);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB,numSlots*collisionBox->getParticles().size()*sizeof(Vertex),0,GL_DYNAMIC_DRAW);
	
	/* Upload initial particle positions into each ring buffer slot: */
	Vertex* vPtr=static_cast<Vertex*>(glMapBufferARB(GL_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
	for(unsigned int slot=0;slot<numSlots;++slot)
		{
		if(useColorMap)
			{
			/* Color particles by speed: */
			for(MyCollisionBox::ParticleList::const_iterator pIt=collisionBox->getParticles().begin();pIt!=collisionBox->getParticles().end();++pIt,++vPtr)
				{
				vPtr->color=(*colorMap)(pIt->getVelocity().sqr());
				vPtr->position=Vertex::Position(pIt->getPosition());
				}
			}
		else
			{
			/* Draw all particles the same color: */
			for(MyCollisionBox::ParticleList::const_iterator pIt=collisionBox->getParticles().begin();pIt!=collisionBox->getParticles().end();++pIt,++vPtr)
				{
				vPtr->color=Vertex::Color(128,128,128);
				vPtr->position=Vertex::Position(pIt->getPosition());
				}
			}
		}
	glUnmapBufferARB(GL_ARRAY_BUFFER_ARB);
	
	/* Protect the particle state ring buffer: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	}

/* Run the application: */
VRUI_APPLICATION_RUN(CollisionBoxTest)
