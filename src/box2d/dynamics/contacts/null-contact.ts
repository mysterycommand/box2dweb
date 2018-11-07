Box2D.inherit(b2NullContact, Box2D.Dynamics.Contacts.b2Contact);

b2NullContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;

b2NullContact.b2NullContact = function() {
  Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
};

b2NullContact.prototype.b2NullContact = function() {
  this.__super.b2Contact.call(this);
};

b2NullContact.prototype.Evaluate = function() {};
