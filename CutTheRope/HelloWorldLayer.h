//
//  HelloWorldLayer.h
//  CutTheRope
//
//  Created by TurtleKnight on 12-11-28.
//  Copyright __MyCompanyName__ 2012年. All rights reserved.
//


#import <GameKit/GameKit.h>

// When you import this file, you import all the cocos2d classes
#import "cocos2d.h"
#import "Box2D.h"
#import "GLES-Render.h"

//Pixel to metres ratio. Box2D uses metres as the unit for measurement.
//This ratio defines how many pixels correspond to 1 Box2D "metre"
//Box2D is optimized for objects of 1x1 metre therefore it makes sense
//to define the ratio so that your most common object type is 1x1 metre.
#define PTM_RATIO 32

// HelloWorldLayer
@interface HelloWorldLayer : CCLayer 
{
	b2World* world;					// strong ref
    NSMutableArray *_ballBodys;
    NSMutableArray *_balls;
    NSMutableArray *_ropes;
    CCSpriteBatchNode *_ropeSpriteSheet;
    
//    b2Body *_selBallBody;
    b2MouseJoint *_mouseJoint;
    b2Body* groundBody;
}

// returns a CCScene that contains the HelloWorldLayer as the only child
+(CCScene *) scene;

@end
