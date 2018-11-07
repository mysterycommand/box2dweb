// tslint:disable variable-name

import FilterData from './filter-data';
import FixtureDef from './fixture-def';

export default class Fixture {
  public m_filter = new FilterData();

  public m_aabb = new Aabb();
  public m_userData: any = undefined;
  public m_body: any = undefined;
  public m_next: any = undefined;
  public m_shape: any = undefined;
  public m_density = 0.0;
  public m_friction = 0.0;
  public m_restitution = 0.0;

  public GetType = function() {
    return this.m_shape.GetType();
  };

  public GetShape = function() {
    return this.m_shape;
  };

  public SetSensor = function(sensor) {
    if (this.m_isSensor == sensor) return;
    this.m_isSensor = sensor;
    if (this.m_body == null) return;
    var edge = this.m_body.GetContactList();
    while (edge) {
      var contact = edge.contact;
      var fixtureA = contact.GetFixtureA();
      var fixtureB = contact.GetFixtureB();
      if (fixtureA == this || fixtureB == this)
        contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
      edge = edge.next;
    }
  };

  public IsSensor = function() {
    return this.m_isSensor;
  };

  public SetFilterData = function(filter) {
    this.m_filter = filter.Copy();
    if (this.m_body) return;
    var edge = this.m_body.GetContactList();
    while (edge) {
      var contact = edge.contact;
      var fixtureA = contact.GetFixtureA();
      var fixtureB = contact.GetFixtureB();
      if (fixtureA == this || fixtureB == this) contact.FlagForFiltering();
      edge = edge.next;
    }
  };

  public GetFilterData = function() {
    return this.m_filter.Copy();
  };

  public GetBody = function() {
    return this.m_body;
  };

  public GetNext = function() {
    return this.m_next;
  };

  public GetUserData = function() {
    return this.m_userData;
  };

  public SetUserData = function(data) {
    this.m_userData = data;
  };

  public TestPoint = function(p) {
    return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
  };

  public RayCast = function(output, input) {
    return this.m_shape.RayCast(output, input, this.m_body.GetTransform());
  };

  public GetMassData = function(massData) {
    if (massData === undefined) massData = null;
    if (massData == null) {
      massData = new b2MassData();
    }
    this.m_shape.ComputeMass(massData, this.m_density);
    return massData;
  };

  public SetDensity = function(density) {
    if (density === undefined) density = 0;
    this.m_density = density;
  };

  public GetDensity = function() {
    return this.m_density;
  };

  public GetFriction = function() {
    return this.m_friction;
  };

  public SetFriction = function(friction) {
    if (friction === undefined) friction = 0;
    this.m_friction = friction;
  };

  public GetRestitution = function() {
    return this.m_restitution;
  };

  public SetRestitution = function(restitution) {
    if (restitution === undefined) restitution = 0;
    this.m_restitution = restitution;
  };

  public GetAABB = function() {
    return this.m_aabb;
  };

  public Create = function(body, xf, def) {
    this.m_userData = def.userData;
    this.m_friction = def.friction;
    this.m_restitution = def.restitution;
    this.m_body = body;
    this.m_next = null;
    this.m_filter = def.filter.Copy();
    this.m_isSensor = def.isSensor;
    this.m_shape = def.shape.Copy();
    this.m_density = def.density;
  };

  public Destroy = function() {
    this.m_shape = null;
  };

  public CreateProxy = function(broadPhase, xf) {
    this.m_shape.ComputeAABB(this.m_aabb, xf);
    this.m_proxy = broadPhase.CreateProxy(this.m_aabb, this);
  };

  public DestroyProxy = function(broadPhase) {
    if (this.m_proxy == null) {
      return;
    }
    broadPhase.DestroyProxy(this.m_proxy);
    this.m_proxy = null;
  };

  public Synchronize = function(broadPhase, transform1, transform2) {
    if (!this.m_proxy) return;
    var aabb1 = new b2AABB();
    var aabb2 = new b2AABB();
    this.m_shape.ComputeAABB(aabb1, transform1);
    this.m_shape.ComputeAABB(aabb2, transform2);
    this.m_aabb.Combine(aabb1, aabb2);
    var displacement = b2Math.SubtractVV(
      transform2.position,
      transform1.position,
    );
    broadPhase.MoveProxy(this.m_proxy, this.m_aabb, displacement);
  };
}
