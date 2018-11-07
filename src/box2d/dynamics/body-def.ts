import Vec2 from '../common/math/vec2';
import Body from './body';

export default class BodyDef {
  public position = new Vec2();
  public linearVelocity = new Vec2();

  public userData: any = undefined;
  public angle = 0.0;
  public angularVelocity = 0.0;
  public linearDamping = 0.0;
  public angularDamping = 0.0;
  public allowSleep = true;
  public awake = true;
  public fixedRotation = false;
  public bullet = false;
  public type = Body.b2_staticBody;
  public active = true;
  public inertiaScale = 1.0;
}
