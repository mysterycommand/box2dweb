export default function a(a2j, undefined) {
  if (
    !(Object.prototype.defineProperty instanceof Function) &&
    Object.prototype.__defineGetter__ instanceof Function &&
    Object.prototype.__defineSetter__ instanceof Function
  ) {
    Object.defineProperty = function(obj, p, cfg) {
      if (cfg.get instanceof Function) obj.__defineGetter__(p, cfg.get);
      if (cfg.set instanceof Function) obj.__defineSetter__(p, cfg.set);
    };
  }

  function emptyFn() {}
  a2j.inherit = function(cls, base) {
    var tmpCtr = cls;
    emptyFn.prototype = base.prototype;
    cls.prototype = new emptyFn();
    cls.prototype.constructor = tmpCtr;
  };

  a2j.generateCallback = function generateCallback(context, cb) {
    return function() {
      cb.apply(context, arguments);
    };
  };

  a2j.NVector = function NVector(length) {
    if (length === undefined) length = 0;
    var tmp = new Array(length || 0);
    for (var i = 0; i < length; ++i) tmp[i] = 0;
    return tmp;
  };

  a2j.is = function is(o1, o2) {
    if (o1 === null) return false;
    if (o2 instanceof Function && o1 instanceof o2) return true;
    if (
      o1.constructor.__implements != undefined &&
      o1.constructor.__implements[o2]
    )
      return true;
    return false;
  };

  a2j.parseUInt = function(v) {
    return Math.abs(parseInt(v));
  };
}
