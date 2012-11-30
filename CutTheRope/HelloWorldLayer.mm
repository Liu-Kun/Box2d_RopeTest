//
//  HelloWorldLayer.mm
//  CutTheRope
//
//  Created by TurtleKnight on 12-11-28.
//  Copyright __MyCompanyName__ 2012年. All rights reserved.
//

// Import the interfaces
#import "HelloWorldLayer.h"

// Needed to obtain the Navigation Controller
#import "AppDelegate.h"

#import "PhysicsSprite.h"

#import "VRope.h"
#import <set>

#pragma mark - HelloWorldLayer

@interface HelloWorldLayer()
-(void) initPhysics;

@end

@implementation HelloWorldLayer

+(CCScene *) scene
{
	// 'scene' is an autorelease object.
	CCScene *scene = [CCScene node];

	// 'layer' is an autorelease object.
	HelloWorldLayer *layer = [HelloWorldLayer node];
	
	// add layer as a child to scene
	[scene addChild: layer];

	// return the scene
	return scene;
}

-(id) init
{
	if( (self=[super init])) {
		
		self.isTouchEnabled = YES;
		self.isAccelerometerEnabled = YES;
		
		[self initPhysics];
		
        [self initLevel];
   
		[self scheduleUpdate];
	}
	return self;
}

-(void)initLevel{
    CGSize s = [CCDirector sharedDirector].winSize;
    
    _ballBodys = [[NSMutableArray alloc] init];
    _ropes = [[NSMutableArray alloc] init];
    _balls = [[NSMutableArray alloc] init];
    
    b2Body *ballBody1 = [self createBallAt:b2Vec2(s.width*0.45,s.height*0.5)];
    b2Body *ballBody2 = [self createBallAt:b2Vec2(s.width*0.65,s.height*0.5)];
    
    //平移关节
//    b2PrismaticJointDef jointDef;
//    b2Vec2 worldAxis(1.0f, 0.0f);
//    jointDef.collideConnected = true;
//    jointDef.Initialize(ballBody1, groundBody,
//                        ballBody1->GetWorldCenter(), worldAxis);
//    world->CreateJoint(&jointDef);
//    
    
    _ropeSpriteSheet = [CCSpriteBatchNode batchNodeWithFile:@"rope.png"];
    [self addChild:_ropeSpriteSheet];
    [self createRopeWithBodyA:ballBody1 anchorA:b2Vec2(ballBody1->GetPosition().x/PTM_RATIO,ballBody1->GetPosition().y/PTM_RATIO) bodyB:ballBody2 anchorB:b2Vec2(ballBody2->GetPosition().x/PTM_RATIO,ballBody2->GetPosition().y/PTM_RATIO) sag:1.0];
}

-(void) finishedLevel
{
    std::set<b2Body *> toDestroy;
    
    // Destroy every rope and add the objects that should be destroyed
    for (VRope *rope in _ropes)
    {
        [rope removeSprites];
        
        // Don't destroy the ground body...
//        if (rope.joint->GetBodyA() != groundBody)
            toDestroy.insert(rope.joint->GetBodyA());
//        if (rope.joint->GetBodyB() != groundBody)
            toDestroy.insert(rope.joint->GetBodyB());
//
        // Destroy the joint already
        world->DestroyJoint(rope.joint);
    }
    [_ropes removeAllObjects];
    
    // Destroy all the objects
    std::set<b2Body *>::iterator pos;
    for(pos = toDestroy.begin(); pos != toDestroy.end(); ++pos)
    {
        b2Body *body = *pos;
        world->DestroyBody(body);
    }
    for (CCSprite *ball in _balls) {
        [self removeChild:ball cleanup:YES];
    }
    [_balls removeAllObjects];
}

-(void)ccTouchesBegan:(NSSet *)touches withEvent:(UIEvent *)event{
    UITouch *touch = [touches anyObject];
    CGPoint location = [touch locationInView:touch.view];
    location = [[CCDirector sharedDirector] convertToGL:location];
    location = [self convertToNodeSpace:location];
    b2Vec2 worldLocation = b2Vec2(location.x/PTM_RATIO, location.y/PTM_RATIO);
    
    NSValue *value = [_ballBodys objectAtIndex:0];
    b2Body *body = (b2Body *)value.pointerValue;
    b2Fixture *fixture =  body->GetFixtureList();
    
    if(_mouseJoint==nil && fixture->TestPoint(worldLocation)){
        b2MouseJointDef md;
        md.bodyA = groundBody;
        md.bodyB = body;
        md.target = worldLocation;
        md.collideConnected = YES;
        md.maxForce = 1000.0f * body->GetMass(); /////
        
        _mouseJoint = (b2MouseJoint *)world->CreateJoint(&md);
//        body->SetAwake(YES);//have set not allowed sleep
    }
    
    
}

-(void)ccTouchesMoved:(NSSet *)touches withEvent:(UIEvent *)event{
    UITouch *touch = [touches anyObject];
    CGPoint location = [touch locationInView:touch.view];
    CGPoint preLocation = [touch previousLocationInView:touch.view];
    location = [[CCDirector sharedDirector] convertToGL:location];
    location = [self convertToNodeSpace:location];
    preLocation = [[CCDirector sharedDirector] convertToGL:preLocation];
    preLocation = [self convertToNodeSpace:preLocation];
    b2Vec2 worldLocation = b2Vec2(location.x/PTM_RATIO,location.y/PTM_RATIO);
    
    if (_mouseJoint) {
        _mouseJoint->SetTarget(worldLocation);

    }else{
        //check segment intersect and cut the rope
        for (VRope *rope in _ropes)
        {
            for (VStick *stick in rope.sticks)
            {
                CGPoint pa = [[stick getPointA] point];
                CGPoint pb = [[stick getPointB] point];
                
                if (ccpSegmentIntersect(preLocation, location, pa, pb))
                {
                    // Cut the rope here
                    b2Body *newBodyA = [self createRopeTipBody];
                    b2Body *newBodyB = [self createRopeTipBody];
                    
                    VRope *newRope = [rope cutRopeInStick:stick newBodyA:newBodyA newBodyB:newBodyB];
                    [_ropes addObject:newRope];
                    
                    return;
                }
            }
        }
    }

}

-(void)ccTouchesCancelled:(NSSet *)touches withEvent:(UIEvent *)event{
    if (_mouseJoint) {
        world->DestroyJoint(_mouseJoint);
        _mouseJoint=nil;
    }
}

-(void)ccTouchesEnded:(NSSet *)touches withEvent:(UIEvent *)event{
    if (_mouseJoint) {
        world->DestroyJoint(_mouseJoint);
        _mouseJoint=nil;
    }
}

-(void) createRopeWithBodyA:(b2Body*)bodyA anchorA:(b2Vec2)anchorA
                      bodyB:(b2Body*)bodyB anchorB:(b2Vec2)anchorB
                        sag:(float32)sag
{
    b2RopeJointDef jd;
    jd.bodyA = bodyA;
    jd.bodyB = bodyB;
    jd.localAnchorA = anchorA;
    jd.localAnchorB = anchorB;
    jd.collideConnected = YES;
    // Max length of joint = current distance between bodies * sag
    float32 ropeLength = (bodyA->GetWorldPoint(anchorA) - bodyB->GetWorldPoint(anchorB)).Length() * sag;

    jd.maxLength = ropeLength;
    
    // Create joint
    b2RopeJoint *ropeJoint = (b2RopeJoint *)world->CreateJoint(&jd);
    
    VRope *newRope = [[VRope alloc] initWithRopeJoint:ropeJoint spriteSheet:_ropeSpriteSheet];
    
    [_ropes addObject:newRope];
    [newRope release];
}

-(b2Body *) createRopeTipBody
{
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.linearDamping = 0.5f;
    b2Body *body = world->CreateBody(&bodyDef);
    
    b2FixtureDef circleDef;
    b2CircleShape circle;
    circle.m_radius = 1.0/PTM_RATIO;
    circleDef.shape = &circle;
    circleDef.density = 1.0f;

    // Since these tips don't have to collide with anything
    // set the mask bits to zero
    circleDef.filter.maskBits = 0;
    body->CreateFixture(&circleDef);
    
    return body;
}

//-(void)giveVelocity{
//
//    NSValue *value = [_ballBodys objectAtIndex:0];
//    b2Body *body = (b2Body*)value.pointerValue;
//    body->SetLinearVelocity(b2Vec2(100.0,0));
//    
//    NSValue *value2 = [_ballBodys objectAtIndex:1];
//    b2Body *body2 = (b2Body*)value2.pointerValue;
//    body2->SetLinearVelocity(b2Vec2(10.0,0));
//}

-(b2Body *)createBallAt:(b2Vec2)location{

    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(location.x/PTM_RATIO, location.y/PTM_RATIO);
    b2Body *body = world->CreateBody(&bodyDef);
    
    b2CircleShape shape;
    shape.m_radius = 26.0/PTM_RATIO;
    b2FixtureDef fixDef;
    fixDef.shape = &shape;
    fixDef.density = 1.0;
    fixDef.friction = 0.2;
    fixDef.restitution = 0.8f;
    body->CreateFixture(&fixDef);
    
    body->SetLinearDamping(0.5);
    body->SetAngularDamping(0.8);
    
    PhysicsSprite *sprite = [PhysicsSprite spriteWithFile:@"Ball.jpg"];
    sprite.position = ccp(location.x, location.y);
    [self addChild:sprite];
    [sprite setPhysicsBody:body];
    
    [_ballBodys addObject:[NSValue valueWithPointer:body]];
    [_balls addObject:sprite];

    return body;
}

-(void) dealloc
{
	delete world;
	world = NULL;
	[_ballBodys release];
    [_ropes release];
    [_balls release];
	[super dealloc];
}	

-(void) initPhysics
{
	
	CGSize s = [[CCDirector sharedDirector] winSize];
	
	b2Vec2 gravity;
	gravity.Set(0.0f, -9.8f);
	world = new b2World(gravity);
	
	
	// Do we want to let bodies sleep?
	world->SetAllowSleeping(NO);
	
	
	// Define the ground body.
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0, 0); // bottom-left corner

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	groundBody = world->CreateBody(&groundBodyDef);
	
	// Define the ground box shape.
	b2EdgeShape groundBox;		
	
	// bottom
	
	groundBox.Set(b2Vec2(0,0), b2Vec2(s.width/PTM_RATIO,0));
	groundBody->CreateFixture(&groundBox,0);
	
	// top
	groundBox.Set(b2Vec2(0,s.height/PTM_RATIO), b2Vec2(s.width/PTM_RATIO,s.height/PTM_RATIO));
	groundBody->CreateFixture(&groundBox,0);
	
	// left
	groundBox.Set(b2Vec2(0,s.height/PTM_RATIO), b2Vec2(0,0));
	groundBody->CreateFixture(&groundBox,0);
	
	// right
	groundBox.Set(b2Vec2(s.width/PTM_RATIO,s.height/PTM_RATIO), b2Vec2(s.width/PTM_RATIO,0));
	groundBody->CreateFixture(&groundBox,0);


}

-(void) update: (ccTime) dt
{
	//It is recommended that a fixed time step is used with Box2D for stability
	//of the simulation, however, we are using a variable time step here.
	//You need to make an informed choice, the following URL is useful
	//http://gafferongames.com/game-physics/fix-your-timestep/
	
	int32 velocityIterations = 8;
	int32 positionIterations = 1;
	
	// Instruct the world to perform a single step of simulation. It is
	// generally best to keep the time step and iterations fixed.
	world->Step(dt, velocityIterations, positionIterations);
    
//    for (b2Body* b = world->GetBodyList(); b; b = b->GetNext())
//	{
//        CCSprite *myActor = (CCSprite*)b->GetUserData();
//		if (myActor)
//        {
//            //Synchronize the AtlasSprites position and rotation with the corresponding body
//            myActor.position = CGPointMake( b->GetPosition().x * PTM_RATIO, b->GetPosition().y * PTM_RATIO);
//            myActor.rotation = -1 * CC_RADIANS_TO_DEGREES(b->GetAngle());
//		}
//	}

    for (VRope *rope in _ropes)
    {
        [rope update:dt];
        [rope updateSprites];
    }
    

}




@end
