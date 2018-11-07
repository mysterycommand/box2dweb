/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
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

import b from './b';
import c from './c';
import d from './d';
import g from './g';
import h from './h';
import i from './i';
import j from './j';

// tslint:disable-next-line no-empty
const Ctor = (function inheritor() {} as any) as { new (): any };

// package structure
export const Box2D = {
  Collision: {
    b2AABB: {},
    Shapes: { b2PolygonShape: {}, b2CircleShape: {} },
  },

  Common: {
    Math: {
      b2Vec2: {},
    },
  },

  Dynamics: {
    b2BodyDef: {},
    b2Body: {},
    b2FixtureDef: {},
    b2World: {},
    b2DebugDraw: {},

    Contacts: {},
    Controllers: {},
    Joints: { b2MouseJointDef: {} },
  },

  postDefs: [],

  inherit(ctor: () => void, base: () => void) {
    const tempCtor = ctor;
    Ctor.prototype = base.prototype;
    ctor.prototype = new Ctor();
    ctor.prototype.constructor = tempCtor;
  },

  generateCallback(context: any, cb: () => void) {
    return (...args: any[]) => {
      cb.apply(context, args);
    };
  },

  NVector(length = 0) {
    return new Array(length).fill(0);
  },

  is(o1: any, o2: any) {
    if (o1 === null) {
      return false;
    }

    if (o2 instanceof Function && o1 instanceof o2) {
      return true;
    }

    if (
      o1.constructor.__implements !== undefined &&
      o1.constructor.__implements[o2]
    ) {
      return true;
    }

    return false;
  },

  parseUInt(v: number) {
    return Math.abs(parseInt(`${v}`, 10));
  },
};

// pre-definitions
b(Box2D);

// definitions
c(Box2D);
d(Box2D);
g(Box2D);
h(Box2D);
i(Box2D);
j(Box2D);

// post-definitions
for (const postDef of Box2D.postDefs as Array<() => void>) {
  postDef();
}
delete Box2D.postDefs;

console.log(Box2D); // tslint:disable-line no-console
