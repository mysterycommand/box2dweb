export default function a(a2j, undefined) {
  a2j.generateCallback = function generateCallback(context, cb) {
    return function() {
      cb.apply(context, arguments);
    };
  };
}
