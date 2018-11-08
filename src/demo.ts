import { Box2D } from './box2d';

const scale = 1 / 29.075;

const {
  Common: {
    Math: { b2Vec2: Vec2 },
  },
  Collision: {
    b2AABB: Aabb,
    Shapes: { b2PolygonShape: PolygonShape, b2CircleShape: CircleShape },
  },
  Dynamics: {
    b2BodyDef: BodyDef,
    b2Body: Body,
    b2FixtureDef: FixtureDef,
    b2World: World,
    b2DebugDraw: DebugDraw,
    Joints: { b2MouseJointDef: MouseJointDef },
  },
} = Box2D;

const c = document.getElementById('canvas') as HTMLCanvasElement;
const ctx = c.getContext('2d');

function init() {
  const world = new World(
    new Vec2(0, 30), // gravity
    true, // allow sleep
  );

  const fixDef = new FixtureDef();
  fixDef.density = 1.0;
  fixDef.friction = 0.5;
  fixDef.restitution = 0.2;

  const bodyDef = new BodyDef();

  // create ground
  bodyDef.type = Body.b2_staticBody;
  fixDef.shape = new PolygonShape();

  fixDef.shape.SetAsBox(c.width * scale, 1);

  bodyDef.position.Set(0, c.height * scale);
  world.CreateBody(bodyDef).CreateFixture(fixDef);

  bodyDef.position.Set(0, -1);
  world.CreateBody(bodyDef).CreateFixture(fixDef);

  fixDef.shape.SetAsBox(1, c.height * scale);

  bodyDef.position.Set(-1, 0);
  world.CreateBody(bodyDef).CreateFixture(fixDef);

  bodyDef.position.Set(c.width * scale, 0);
  world.CreateBody(bodyDef).CreateFixture(fixDef);

  // create some objects
  bodyDef.type = Body.b2_dynamicBody;
  for (let i = 0; i < 10; ++i) {
    if (Math.random() > 0.5) {
      fixDef.shape = new PolygonShape();
      fixDef.shape.SetAsBox(
        Math.random() + 0.1, // half width
        Math.random() + 0.1, // half height
      );
    } else {
      fixDef.shape = new CircleShape(
        Math.random() + 0.1, // radius
      );
    }
    bodyDef.position.x = Math.random() * 10;
    bodyDef.position.y = Math.random() * 10;
    world.CreateBody(bodyDef).CreateFixture(fixDef);
  }

  // setup debug draw
  const debugDraw = new DebugDraw();
  debugDraw.SetSprite(ctx);
  debugDraw.SetDrawScale(30.0);
  debugDraw.SetFillAlpha(0.5);
  debugDraw.SetLineThickness(1.0);
  debugDraw.SetFlags(DebugDraw.e_shapeBit | DebugDraw.e_jointBit);
  world.SetDebugDraw(debugDraw);

  window.setInterval(update, 1000 / 60);

  // mouse
  let mouseX: number = c.width / 2;
  let mouseY: number = c.height / 2;
  let mousePVec: any;
  let isMouseDown: boolean;
  let selectedBody: any;
  let mouseJoint: any;

  const canvasPosition = c.getBoundingClientRect() as DOMRect;

  function handleMouseDown(event: MouseEvent | TouchEvent) {
    isMouseDown = true;
    handleMouseMove(event);

    document.addEventListener('mousemove', handleMouseMove, true);
    document.addEventListener('touchmove', handleMouseMove, true);
  }

  document.addEventListener('mousedown', handleMouseDown, true);
  document.addEventListener('touchstart', handleMouseDown, true);

  function handleMouseUp() {
    document.removeEventListener('mousemove', handleMouseMove, true);
    document.removeEventListener('touchmove', handleMouseMove, true);

    isMouseDown = false;

    mouseX = c.width / 2;
    mouseY = c.height / 2;
  }

  document.addEventListener('mouseup', handleMouseUp, true);
  document.addEventListener('touchend', handleMouseUp, true);

  const isMouseEvent = (event: any): event is MouseEvent => {
    return event.type.includes('mouse');
  };

  const isTouchEvent = (event: any): event is TouchEvent => {
    return event.type.includes('touch');
  };

  function handleMouseMove(event: MouseEvent | TouchEvent) {
    event.preventDefault();
    event.stopPropagation();

    let clientX;
    let clientY;

    if (isMouseEvent(event)) {
      clientX = event.clientX;
      clientY = event.clientY;
    } else if (isTouchEvent(event)) {
      // TODO: @mysterycommand - not sure if we always have changedTouches
      const touch = event.changedTouches[event.changedTouches.length - 1];

      clientX = touch.clientX;
      clientY = touch.clientY;
    } else {
      return;
    }

    mouseX = (clientX - canvasPosition.x) / 30;
    mouseY = (clientY - canvasPosition.y) / 30;
  }

  function getBodyAtMouse() {
    mousePVec = new Vec2(mouseX, mouseY);
    const aabb = new Aabb();

    aabb.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
    aabb.upperBound.Set(mouseX + 0.001, mouseY + 0.001);

    // Query the world for overlapping shapes.
    selectedBody = null;
    world.QueryAABB(getBodyCB, aabb);
    return selectedBody;
  }

  function getBodyCB(fixture: any) {
    if (fixture.GetBody().GetType() !== Body.b2_staticBody) {
      if (
        fixture
          .GetShape()
          .TestPoint(fixture.GetBody().GetTransform(), mousePVec)
      ) {
        selectedBody = fixture.GetBody();
        return false;
      }
    }
    return true;
  }

  // update
  function update() {
    if (isMouseDown && !mouseJoint) {
      const body = getBodyAtMouse();

      if (body) {
        const md = new MouseJointDef();
        md.bodyA = world.GetGroundBody();
        md.bodyB = body;
        md.target.Set(mouseX, mouseY);
        md.collideConnected = true;
        md.maxForce = 300.0 * body.GetMass();
        mouseJoint = world.CreateJoint(md);
        body.SetAwake(true);
      }
    }

    if (mouseJoint) {
      if (isMouseDown) {
        mouseJoint.SetTarget(new Vec2(mouseX, mouseY));
      } else {
        world.DestroyJoint(mouseJoint);
        mouseJoint = null;
      }
    }

    world.Step(1 / 60, 10, 10);
    world.DrawDebugData();
    world.ClearForces();
  }
}

addEventListener('load', ({ target }) => {
  const { innerWidth: w, innerHeight: h } = window;

  c.width = w;
  c.style.width = `${w}px`;

  c.height = h;
  c.style.height = `${h}px`;

  init();
});
