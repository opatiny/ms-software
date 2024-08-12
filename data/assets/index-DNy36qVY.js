(function () {
  const t = document.createElement("link").relList;
  if (t && t.supports && t.supports("modulepreload")) return;
  for (const i of document.querySelectorAll('link[rel="modulepreload"]')) r(i);
  new MutationObserver((i) => {
    for (const o of i)
      if (o.type === "childList")
        for (const l of o.addedNodes)
          l.tagName === "LINK" && l.rel === "modulepreload" && r(l);
  }).observe(document, { childList: !0, subtree: !0 });
  function n(i) {
    const o = {};
    return (
      i.integrity && (o.integrity = i.integrity),
      i.referrerPolicy && (o.referrerPolicy = i.referrerPolicy),
      i.crossOrigin === "use-credentials"
        ? (o.credentials = "include")
        : i.crossOrigin === "anonymous"
        ? (o.credentials = "omit")
        : (o.credentials = "same-origin"),
      o
    );
  }
  function r(i) {
    if (i.ep) return;
    i.ep = !0;
    const o = n(i);
    fetch(i.href, o);
  }
})();
function r0(e) {
  return e && e.__esModule && Object.prototype.hasOwnProperty.call(e, "default")
    ? e.default
    : e;
}
var Xf = { exports: {} },
  el = {},
  Kf = { exports: {} },
  I = {};
/**
 * @license React
 * react.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */ var Ci = Symbol.for("react.element"),
  i0 = Symbol.for("react.portal"),
  o0 = Symbol.for("react.fragment"),
  l0 = Symbol.for("react.strict_mode"),
  s0 = Symbol.for("react.profiler"),
  u0 = Symbol.for("react.provider"),
  a0 = Symbol.for("react.context"),
  c0 = Symbol.for("react.forward_ref"),
  f0 = Symbol.for("react.suspense"),
  d0 = Symbol.for("react.memo"),
  h0 = Symbol.for("react.lazy"),
  Oa = Symbol.iterator;
function p0(e) {
  return e === null || typeof e != "object"
    ? null
    : ((e = (Oa && e[Oa]) || e["@@iterator"]),
      typeof e == "function" ? e : null);
}
var Zf = {
    isMounted: function () {
      return !1;
    },
    enqueueForceUpdate: function () {},
    enqueueReplaceState: function () {},
    enqueueSetState: function () {},
  },
  Jf = Object.assign,
  qf = {};
function gr(e, t, n) {
  (this.props = e),
    (this.context = t),
    (this.refs = qf),
    (this.updater = n || Zf);
}
gr.prototype.isReactComponent = {};
gr.prototype.setState = function (e, t) {
  if (typeof e != "object" && typeof e != "function" && e != null)
    throw Error(
      "setState(...): takes an object of state variables to update or a function which returns an object of state variables."
    );
  this.updater.enqueueSetState(this, e, t, "setState");
};
gr.prototype.forceUpdate = function (e) {
  this.updater.enqueueForceUpdate(this, e, "forceUpdate");
};
function ed() {}
ed.prototype = gr.prototype;
function xu(e, t, n) {
  (this.props = e),
    (this.context = t),
    (this.refs = qf),
    (this.updater = n || Zf);
}
var vu = (xu.prototype = new ed());
vu.constructor = xu;
Jf(vu, gr.prototype);
vu.isPureReactComponent = !0;
var Ra = Array.isArray,
  td = Object.prototype.hasOwnProperty,
  wu = { current: null },
  nd = { key: !0, ref: !0, __self: !0, __source: !0 };
function rd(e, t, n) {
  var r,
    i = {},
    o = null,
    l = null;
  if (t != null)
    for (r in (t.ref !== void 0 && (l = t.ref),
    t.key !== void 0 && (o = "" + t.key),
    t))
      td.call(t, r) && !nd.hasOwnProperty(r) && (i[r] = t[r]);
  var s = arguments.length - 2;
  if (s === 1) i.children = n;
  else if (1 < s) {
    for (var u = Array(s), a = 0; a < s; a++) u[a] = arguments[a + 2];
    i.children = u;
  }
  if (e && e.defaultProps)
    for (r in ((s = e.defaultProps), s)) i[r] === void 0 && (i[r] = s[r]);
  return {
    $$typeof: Ci,
    type: e,
    key: o,
    ref: l,
    props: i,
    _owner: wu.current,
  };
}
function m0(e, t) {
  return {
    $$typeof: Ci,
    type: e.type,
    key: t,
    ref: e.ref,
    props: e.props,
    _owner: e._owner,
  };
}
function Su(e) {
  return typeof e == "object" && e !== null && e.$$typeof === Ci;
}
function y0(e) {
  var t = { "=": "=0", ":": "=2" };
  return (
    "$" +
    e.replace(/[=:]/g, function (n) {
      return t[n];
    })
  );
}
var Fa = /\/+/g;
function jl(e, t) {
  return typeof e == "object" && e !== null && e.key != null
    ? y0("" + e.key)
    : t.toString(36);
}
function ro(e, t, n, r, i) {
  var o = typeof e;
  (o === "undefined" || o === "boolean") && (e = null);
  var l = !1;
  if (e === null) l = !0;
  else
    switch (o) {
      case "string":
      case "number":
        l = !0;
        break;
      case "object":
        switch (e.$$typeof) {
          case Ci:
          case i0:
            l = !0;
        }
    }
  if (l)
    return (
      (l = e),
      (i = i(l)),
      (e = r === "" ? "." + jl(l, 0) : r),
      Ra(i)
        ? ((n = ""),
          e != null && (n = e.replace(Fa, "$&/") + "/"),
          ro(i, t, n, "", function (a) {
            return a;
          }))
        : i != null &&
          (Su(i) &&
            (i = m0(
              i,
              n +
                (!i.key || (l && l.key === i.key)
                  ? ""
                  : ("" + i.key).replace(Fa, "$&/") + "/") +
                e
            )),
          t.push(i)),
      1
    );
  if (((l = 0), (r = r === "" ? "." : r + ":"), Ra(e)))
    for (var s = 0; s < e.length; s++) {
      o = e[s];
      var u = r + jl(o, s);
      l += ro(o, t, n, u, i);
    }
  else if (((u = p0(e)), typeof u == "function"))
    for (e = u.call(e), s = 0; !(o = e.next()).done; )
      (o = o.value), (u = r + jl(o, s++)), (l += ro(o, t, n, u, i));
  else if (o === "object")
    throw (
      ((t = String(e)),
      Error(
        "Objects are not valid as a React child (found: " +
          (t === "[object Object]"
            ? "object with keys {" + Object.keys(e).join(", ") + "}"
            : t) +
          "). If you meant to render a collection of children, use an array instead."
      ))
    );
  return l;
}
function Oi(e, t, n) {
  if (e == null) return e;
  var r = [],
    i = 0;
  return (
    ro(e, r, "", "", function (o) {
      return t.call(n, o, i++);
    }),
    r
  );
}
function g0(e) {
  if (e._status === -1) {
    var t = e._result;
    (t = t()),
      t.then(
        function (n) {
          (e._status === 0 || e._status === -1) &&
            ((e._status = 1), (e._result = n));
        },
        function (n) {
          (e._status === 0 || e._status === -1) &&
            ((e._status = 2), (e._result = n));
        }
      ),
      e._status === -1 && ((e._status = 0), (e._result = t));
  }
  if (e._status === 1) return e._result.default;
  throw e._result;
}
var Me = { current: null },
  io = { transition: null },
  x0 = {
    ReactCurrentDispatcher: Me,
    ReactCurrentBatchConfig: io,
    ReactCurrentOwner: wu,
  };
function id() {
  throw Error("act(...) is not supported in production builds of React.");
}
I.Children = {
  map: Oi,
  forEach: function (e, t, n) {
    Oi(
      e,
      function () {
        t.apply(this, arguments);
      },
      n
    );
  },
  count: function (e) {
    var t = 0;
    return (
      Oi(e, function () {
        t++;
      }),
      t
    );
  },
  toArray: function (e) {
    return (
      Oi(e, function (t) {
        return t;
      }) || []
    );
  },
  only: function (e) {
    if (!Su(e))
      throw Error(
        "React.Children.only expected to receive a single React element child."
      );
    return e;
  },
};
I.Component = gr;
I.Fragment = o0;
I.Profiler = s0;
I.PureComponent = xu;
I.StrictMode = l0;
I.Suspense = f0;
I.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED = x0;
I.act = id;
I.cloneElement = function (e, t, n) {
  if (e == null)
    throw Error(
      "React.cloneElement(...): The argument must be a React element, but you passed " +
        e +
        "."
    );
  var r = Jf({}, e.props),
    i = e.key,
    o = e.ref,
    l = e._owner;
  if (t != null) {
    if (
      (t.ref !== void 0 && ((o = t.ref), (l = wu.current)),
      t.key !== void 0 && (i = "" + t.key),
      e.type && e.type.defaultProps)
    )
      var s = e.type.defaultProps;
    for (u in t)
      td.call(t, u) &&
        !nd.hasOwnProperty(u) &&
        (r[u] = t[u] === void 0 && s !== void 0 ? s[u] : t[u]);
  }
  var u = arguments.length - 2;
  if (u === 1) r.children = n;
  else if (1 < u) {
    s = Array(u);
    for (var a = 0; a < u; a++) s[a] = arguments[a + 2];
    r.children = s;
  }
  return { $$typeof: Ci, type: e.type, key: i, ref: o, props: r, _owner: l };
};
I.createContext = function (e) {
  return (
    (e = {
      $$typeof: a0,
      _currentValue: e,
      _currentValue2: e,
      _threadCount: 0,
      Provider: null,
      Consumer: null,
      _defaultValue: null,
      _globalName: null,
    }),
    (e.Provider = { $$typeof: u0, _context: e }),
    (e.Consumer = e)
  );
};
I.createElement = rd;
I.createFactory = function (e) {
  var t = rd.bind(null, e);
  return (t.type = e), t;
};
I.createRef = function () {
  return { current: null };
};
I.forwardRef = function (e) {
  return { $$typeof: c0, render: e };
};
I.isValidElement = Su;
I.lazy = function (e) {
  return { $$typeof: h0, _payload: { _status: -1, _result: e }, _init: g0 };
};
I.memo = function (e, t) {
  return { $$typeof: d0, type: e, compare: t === void 0 ? null : t };
};
I.startTransition = function (e) {
  var t = io.transition;
  io.transition = {};
  try {
    e();
  } finally {
    io.transition = t;
  }
};
I.unstable_act = id;
I.useCallback = function (e, t) {
  return Me.current.useCallback(e, t);
};
I.useContext = function (e) {
  return Me.current.useContext(e);
};
I.useDebugValue = function () {};
I.useDeferredValue = function (e) {
  return Me.current.useDeferredValue(e);
};
I.useEffect = function (e, t) {
  return Me.current.useEffect(e, t);
};
I.useId = function () {
  return Me.current.useId();
};
I.useImperativeHandle = function (e, t, n) {
  return Me.current.useImperativeHandle(e, t, n);
};
I.useInsertionEffect = function (e, t) {
  return Me.current.useInsertionEffect(e, t);
};
I.useLayoutEffect = function (e, t) {
  return Me.current.useLayoutEffect(e, t);
};
I.useMemo = function (e, t) {
  return Me.current.useMemo(e, t);
};
I.useReducer = function (e, t, n) {
  return Me.current.useReducer(e, t, n);
};
I.useRef = function (e) {
  return Me.current.useRef(e);
};
I.useState = function (e) {
  return Me.current.useState(e);
};
I.useSyncExternalStore = function (e, t, n) {
  return Me.current.useSyncExternalStore(e, t, n);
};
I.useTransition = function () {
  return Me.current.useTransition();
};
I.version = "18.3.1";
Kf.exports = I;
var _ = Kf.exports;
const v0 = r0(_);
/**
 * @license React
 * react-jsx-runtime.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */ var w0 = _,
  S0 = Symbol.for("react.element"),
  k0 = Symbol.for("react.fragment"),
  j0 = Object.prototype.hasOwnProperty,
  E0 = w0.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED.ReactCurrentOwner,
  P0 = { key: !0, ref: !0, __self: !0, __source: !0 };
function od(e, t, n) {
  var r,
    i = {},
    o = null,
    l = null;
  n !== void 0 && (o = "" + n),
    t.key !== void 0 && (o = "" + t.key),
    t.ref !== void 0 && (l = t.ref);
  for (r in t) j0.call(t, r) && !P0.hasOwnProperty(r) && (i[r] = t[r]);
  if (e && e.defaultProps)
    for (r in ((t = e.defaultProps), t)) i[r] === void 0 && (i[r] = t[r]);
  return {
    $$typeof: S0,
    type: e,
    key: o,
    ref: l,
    props: i,
    _owner: E0.current,
  };
}
el.Fragment = k0;
el.jsx = od;
el.jsxs = od;
Xf.exports = el;
var c = Xf.exports,
  cs = {},
  ld = { exports: {} },
  Ye = {},
  sd = { exports: {} },
  ud = {};
/**
 * @license React
 * scheduler.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */ (function (e) {
  function t(T, L) {
    var z = T.length;
    T.push(L);
    e: for (; 0 < z; ) {
      var F = (z - 1) >>> 1,
        X = T[F];
      if (0 < i(X, L)) (T[F] = L), (T[z] = X), (z = F);
      else break e;
    }
  }
  function n(T) {
    return T.length === 0 ? null : T[0];
  }
  function r(T) {
    if (T.length === 0) return null;
    var L = T[0],
      z = T.pop();
    if (z !== L) {
      T[0] = z;
      e: for (var F = 0, X = T.length, _e = X >>> 1; F < _e; ) {
        var Ge = 2 * (F + 1) - 1,
          nn = T[Ge],
          wt = Ge + 1,
          $n = T[wt];
        if (0 > i(nn, z))
          wt < X && 0 > i($n, nn)
            ? ((T[F] = $n), (T[wt] = z), (F = wt))
            : ((T[F] = nn), (T[Ge] = z), (F = Ge));
        else if (wt < X && 0 > i($n, z)) (T[F] = $n), (T[wt] = z), (F = wt);
        else break e;
      }
    }
    return L;
  }
  function i(T, L) {
    var z = T.sortIndex - L.sortIndex;
    return z !== 0 ? z : T.id - L.id;
  }
  if (typeof performance == "object" && typeof performance.now == "function") {
    var o = performance;
    e.unstable_now = function () {
      return o.now();
    };
  } else {
    var l = Date,
      s = l.now();
    e.unstable_now = function () {
      return l.now() - s;
    };
  }
  var u = [],
    a = [],
    f = 1,
    d = null,
    h = 3,
    x = !1,
    g = !1,
    v = !1,
    j = typeof setTimeout == "function" ? setTimeout : null,
    m = typeof clearTimeout == "function" ? clearTimeout : null,
    p = typeof setImmediate < "u" ? setImmediate : null;
  typeof navigator < "u" &&
    navigator.scheduling !== void 0 &&
    navigator.scheduling.isInputPending !== void 0 &&
    navigator.scheduling.isInputPending.bind(navigator.scheduling);
  function y(T) {
    for (var L = n(a); L !== null; ) {
      if (L.callback === null) r(a);
      else if (L.startTime <= T)
        r(a), (L.sortIndex = L.expirationTime), t(u, L);
      else break;
      L = n(a);
    }
  }
  function w(T) {
    if (((v = !1), y(T), !g))
      if (n(u) !== null) (g = !0), b(S);
      else {
        var L = n(a);
        L !== null && le(w, L.startTime - T);
      }
  }
  function S(T, L) {
    (g = !1), v && ((v = !1), m(P), (P = -1)), (x = !0);
    var z = h;
    try {
      for (
        y(L), d = n(u);
        d !== null && (!(d.expirationTime > L) || (T && !N()));

      ) {
        var F = d.callback;
        if (typeof F == "function") {
          (d.callback = null), (h = d.priorityLevel);
          var X = F(d.expirationTime <= L);
          (L = e.unstable_now()),
            typeof X == "function" ? (d.callback = X) : d === n(u) && r(u),
            y(L);
        } else r(u);
        d = n(u);
      }
      if (d !== null) var _e = !0;
      else {
        var Ge = n(a);
        Ge !== null && le(w, Ge.startTime - L), (_e = !1);
      }
      return _e;
    } finally {
      (d = null), (h = z), (x = !1);
    }
  }
  var k = !1,
    E = null,
    P = -1,
    D = 5,
    O = -1;
  function N() {
    return !(e.unstable_now() - O < D);
  }
  function W() {
    if (E !== null) {
      var T = e.unstable_now();
      O = T;
      var L = !0;
      try {
        L = E(!0, T);
      } finally {
        L ? R() : ((k = !1), (E = null));
      }
    } else k = !1;
  }
  var R;
  if (typeof p == "function")
    R = function () {
      p(W);
    };
  else if (typeof MessageChannel < "u") {
    var Q = new MessageChannel(),
      K = Q.port2;
    (Q.port1.onmessage = W),
      (R = function () {
        K.postMessage(null);
      });
  } else
    R = function () {
      j(W, 0);
    };
  function b(T) {
    (E = T), k || ((k = !0), R());
  }
  function le(T, L) {
    P = j(function () {
      T(e.unstable_now());
    }, L);
  }
  (e.unstable_IdlePriority = 5),
    (e.unstable_ImmediatePriority = 1),
    (e.unstable_LowPriority = 4),
    (e.unstable_NormalPriority = 3),
    (e.unstable_Profiling = null),
    (e.unstable_UserBlockingPriority = 2),
    (e.unstable_cancelCallback = function (T) {
      T.callback = null;
    }),
    (e.unstable_continueExecution = function () {
      g || x || ((g = !0), b(S));
    }),
    (e.unstable_forceFrameRate = function (T) {
      0 > T || 125 < T
        ? console.error(
            "forceFrameRate takes a positive int between 0 and 125, forcing frame rates higher than 125 fps is not supported"
          )
        : (D = 0 < T ? Math.floor(1e3 / T) : 5);
    }),
    (e.unstable_getCurrentPriorityLevel = function () {
      return h;
    }),
    (e.unstable_getFirstCallbackNode = function () {
      return n(u);
    }),
    (e.unstable_next = function (T) {
      switch (h) {
        case 1:
        case 2:
        case 3:
          var L = 3;
          break;
        default:
          L = h;
      }
      var z = h;
      h = L;
      try {
        return T();
      } finally {
        h = z;
      }
    }),
    (e.unstable_pauseExecution = function () {}),
    (e.unstable_requestPaint = function () {}),
    (e.unstable_runWithPriority = function (T, L) {
      switch (T) {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
          break;
        default:
          T = 3;
      }
      var z = h;
      h = T;
      try {
        return L();
      } finally {
        h = z;
      }
    }),
    (e.unstable_scheduleCallback = function (T, L, z) {
      var F = e.unstable_now();
      switch (
        (typeof z == "object" && z !== null
          ? ((z = z.delay), (z = typeof z == "number" && 0 < z ? F + z : F))
          : (z = F),
        T)
      ) {
        case 1:
          var X = -1;
          break;
        case 2:
          X = 250;
          break;
        case 5:
          X = 1073741823;
          break;
        case 4:
          X = 1e4;
          break;
        default:
          X = 5e3;
      }
      return (
        (X = z + X),
        (T = {
          id: f++,
          callback: L,
          priorityLevel: T,
          startTime: z,
          expirationTime: X,
          sortIndex: -1,
        }),
        z > F
          ? ((T.sortIndex = z),
            t(a, T),
            n(u) === null &&
              T === n(a) &&
              (v ? (m(P), (P = -1)) : (v = !0), le(w, z - F)))
          : ((T.sortIndex = X), t(u, T), g || x || ((g = !0), b(S))),
        T
      );
    }),
    (e.unstable_shouldYield = N),
    (e.unstable_wrapCallback = function (T) {
      var L = h;
      return function () {
        var z = h;
        h = L;
        try {
          return T.apply(this, arguments);
        } finally {
          h = z;
        }
      };
    });
})(ud);
sd.exports = ud;
var C0 = sd.exports;
/**
 * @license React
 * react-dom.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */ var M0 = _,
  We = C0;
function C(e) {
  for (
    var t = "https://reactjs.org/docs/error-decoder.html?invariant=" + e, n = 1;
    n < arguments.length;
    n++
  )
    t += "&args[]=" + encodeURIComponent(arguments[n]);
  return (
    "Minified React error #" +
    e +
    "; visit " +
    t +
    " for the full message or use the non-minified dev environment for full errors and additional helpful warnings."
  );
}
var ad = new Set(),
  ni = {};
function Tn(e, t) {
  lr(e, t), lr(e + "Capture", t);
}
function lr(e, t) {
  for (ni[e] = t, e = 0; e < t.length; e++) ad.add(t[e]);
}
var Tt = !(
    typeof window > "u" ||
    typeof window.document > "u" ||
    typeof window.document.createElement > "u"
  ),
  fs = Object.prototype.hasOwnProperty,
  T0 =
    /^[:A-Z_a-z\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD][:A-Z_a-z\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD\-.0-9\u00B7\u0300-\u036F\u203F-\u2040]*$/,
  Ia = {},
  Ua = {};
function _0(e) {
  return fs.call(Ua, e)
    ? !0
    : fs.call(Ia, e)
    ? !1
    : T0.test(e)
    ? (Ua[e] = !0)
    : ((Ia[e] = !0), !1);
}
function L0(e, t, n, r) {
  if (n !== null && n.type === 0) return !1;
  switch (typeof t) {
    case "function":
    case "symbol":
      return !0;
    case "boolean":
      return r
        ? !1
        : n !== null
        ? !n.acceptsBooleans
        : ((e = e.toLowerCase().slice(0, 5)), e !== "data-" && e !== "aria-");
    default:
      return !1;
  }
}
function N0(e, t, n, r) {
  if (t === null || typeof t > "u" || L0(e, t, n, r)) return !0;
  if (r) return !1;
  if (n !== null)
    switch (n.type) {
      case 3:
        return !t;
      case 4:
        return t === !1;
      case 5:
        return isNaN(t);
      case 6:
        return isNaN(t) || 1 > t;
    }
  return !1;
}
function Te(e, t, n, r, i, o, l) {
  (this.acceptsBooleans = t === 2 || t === 3 || t === 4),
    (this.attributeName = r),
    (this.attributeNamespace = i),
    (this.mustUseProperty = n),
    (this.propertyName = e),
    (this.type = t),
    (this.sanitizeURL = o),
    (this.removeEmptyString = l);
}
var ve = {};
"children dangerouslySetInnerHTML defaultValue defaultChecked innerHTML suppressContentEditableWarning suppressHydrationWarning style"
  .split(" ")
  .forEach(function (e) {
    ve[e] = new Te(e, 0, !1, e, null, !1, !1);
  });
[
  ["acceptCharset", "accept-charset"],
  ["className", "class"],
  ["htmlFor", "for"],
  ["httpEquiv", "http-equiv"],
].forEach(function (e) {
  var t = e[0];
  ve[t] = new Te(t, 1, !1, e[1], null, !1, !1);
});
["contentEditable", "draggable", "spellCheck", "value"].forEach(function (e) {
  ve[e] = new Te(e, 2, !1, e.toLowerCase(), null, !1, !1);
});
[
  "autoReverse",
  "externalResourcesRequired",
  "focusable",
  "preserveAlpha",
].forEach(function (e) {
  ve[e] = new Te(e, 2, !1, e, null, !1, !1);
});
"allowFullScreen async autoFocus autoPlay controls default defer disabled disablePictureInPicture disableRemotePlayback formNoValidate hidden loop noModule noValidate open playsInline readOnly required reversed scoped seamless itemScope"
  .split(" ")
  .forEach(function (e) {
    ve[e] = new Te(e, 3, !1, e.toLowerCase(), null, !1, !1);
  });
["checked", "multiple", "muted", "selected"].forEach(function (e) {
  ve[e] = new Te(e, 3, !0, e, null, !1, !1);
});
["capture", "download"].forEach(function (e) {
  ve[e] = new Te(e, 4, !1, e, null, !1, !1);
});
["cols", "rows", "size", "span"].forEach(function (e) {
  ve[e] = new Te(e, 6, !1, e, null, !1, !1);
});
["rowSpan", "start"].forEach(function (e) {
  ve[e] = new Te(e, 5, !1, e.toLowerCase(), null, !1, !1);
});
var ku = /[\-:]([a-z])/g;
function ju(e) {
  return e[1].toUpperCase();
}
"accent-height alignment-baseline arabic-form baseline-shift cap-height clip-path clip-rule color-interpolation color-interpolation-filters color-profile color-rendering dominant-baseline enable-background fill-opacity fill-rule flood-color flood-opacity font-family font-size font-size-adjust font-stretch font-style font-variant font-weight glyph-name glyph-orientation-horizontal glyph-orientation-vertical horiz-adv-x horiz-origin-x image-rendering letter-spacing lighting-color marker-end marker-mid marker-start overline-position overline-thickness paint-order panose-1 pointer-events rendering-intent shape-rendering stop-color stop-opacity strikethrough-position strikethrough-thickness stroke-dasharray stroke-dashoffset stroke-linecap stroke-linejoin stroke-miterlimit stroke-opacity stroke-width text-anchor text-decoration text-rendering underline-position underline-thickness unicode-bidi unicode-range units-per-em v-alphabetic v-hanging v-ideographic v-mathematical vector-effect vert-adv-y vert-origin-x vert-origin-y word-spacing writing-mode xmlns:xlink x-height"
  .split(" ")
  .forEach(function (e) {
    var t = e.replace(ku, ju);
    ve[t] = new Te(t, 1, !1, e, null, !1, !1);
  });
"xlink:actuate xlink:arcrole xlink:role xlink:show xlink:title xlink:type"
  .split(" ")
  .forEach(function (e) {
    var t = e.replace(ku, ju);
    ve[t] = new Te(t, 1, !1, e, "http://www.w3.org/1999/xlink", !1, !1);
  });
["xml:base", "xml:lang", "xml:space"].forEach(function (e) {
  var t = e.replace(ku, ju);
  ve[t] = new Te(t, 1, !1, e, "http://www.w3.org/XML/1998/namespace", !1, !1);
});
["tabIndex", "crossOrigin"].forEach(function (e) {
  ve[e] = new Te(e, 1, !1, e.toLowerCase(), null, !1, !1);
});
ve.xlinkHref = new Te(
  "xlinkHref",
  1,
  !1,
  "xlink:href",
  "http://www.w3.org/1999/xlink",
  !0,
  !1
);
["src", "href", "action", "formAction"].forEach(function (e) {
  ve[e] = new Te(e, 1, !1, e.toLowerCase(), null, !0, !0);
});
function Eu(e, t, n, r) {
  var i = ve.hasOwnProperty(t) ? ve[t] : null;
  (i !== null
    ? i.type !== 0
    : r ||
      !(2 < t.length) ||
      (t[0] !== "o" && t[0] !== "O") ||
      (t[1] !== "n" && t[1] !== "N")) &&
    (N0(t, n, i, r) && (n = null),
    r || i === null
      ? _0(t) && (n === null ? e.removeAttribute(t) : e.setAttribute(t, "" + n))
      : i.mustUseProperty
      ? (e[i.propertyName] = n === null ? (i.type === 3 ? !1 : "") : n)
      : ((t = i.attributeName),
        (r = i.attributeNamespace),
        n === null
          ? e.removeAttribute(t)
          : ((i = i.type),
            (n = i === 3 || (i === 4 && n === !0) ? "" : "" + n),
            r ? e.setAttributeNS(r, t, n) : e.setAttribute(t, n))));
}
var Dt = M0.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED,
  Ri = Symbol.for("react.element"),
  Fn = Symbol.for("react.portal"),
  In = Symbol.for("react.fragment"),
  Pu = Symbol.for("react.strict_mode"),
  ds = Symbol.for("react.profiler"),
  cd = Symbol.for("react.provider"),
  fd = Symbol.for("react.context"),
  Cu = Symbol.for("react.forward_ref"),
  hs = Symbol.for("react.suspense"),
  ps = Symbol.for("react.suspense_list"),
  Mu = Symbol.for("react.memo"),
  Ot = Symbol.for("react.lazy"),
  dd = Symbol.for("react.offscreen"),
  Ha = Symbol.iterator;
function Er(e) {
  return e === null || typeof e != "object"
    ? null
    : ((e = (Ha && e[Ha]) || e["@@iterator"]),
      typeof e == "function" ? e : null);
}
var ie = Object.assign,
  El;
function Fr(e) {
  if (El === void 0)
    try {
      throw Error();
    } catch (n) {
      var t = n.stack.trim().match(/\n( *(at )?)/);
      El = (t && t[1]) || "";
    }
  return (
    `
` +
    El +
    e
  );
}
var Pl = !1;
function Cl(e, t) {
  if (!e || Pl) return "";
  Pl = !0;
  var n = Error.prepareStackTrace;
  Error.prepareStackTrace = void 0;
  try {
    if (t)
      if (
        ((t = function () {
          throw Error();
        }),
        Object.defineProperty(t.prototype, "props", {
          set: function () {
            throw Error();
          },
        }),
        typeof Reflect == "object" && Reflect.construct)
      ) {
        try {
          Reflect.construct(t, []);
        } catch (a) {
          var r = a;
        }
        Reflect.construct(e, [], t);
      } else {
        try {
          t.call();
        } catch (a) {
          r = a;
        }
        e.call(t.prototype);
      }
    else {
      try {
        throw Error();
      } catch (a) {
        r = a;
      }
      e();
    }
  } catch (a) {
    if (a && r && typeof a.stack == "string") {
      for (
        var i = a.stack.split(`
`),
          o = r.stack.split(`
`),
          l = i.length - 1,
          s = o.length - 1;
        1 <= l && 0 <= s && i[l] !== o[s];

      )
        s--;
      for (; 1 <= l && 0 <= s; l--, s--)
        if (i[l] !== o[s]) {
          if (l !== 1 || s !== 1)
            do
              if ((l--, s--, 0 > s || i[l] !== o[s])) {
                var u =
                  `
` + i[l].replace(" at new ", " at ");
                return (
                  e.displayName &&
                    u.includes("<anonymous>") &&
                    (u = u.replace("<anonymous>", e.displayName)),
                  u
                );
              }
            while (1 <= l && 0 <= s);
          break;
        }
    }
  } finally {
    (Pl = !1), (Error.prepareStackTrace = n);
  }
  return (e = e ? e.displayName || e.name : "") ? Fr(e) : "";
}
function $0(e) {
  switch (e.tag) {
    case 5:
      return Fr(e.type);
    case 16:
      return Fr("Lazy");
    case 13:
      return Fr("Suspense");
    case 19:
      return Fr("SuspenseList");
    case 0:
    case 2:
    case 15:
      return (e = Cl(e.type, !1)), e;
    case 11:
      return (e = Cl(e.type.render, !1)), e;
    case 1:
      return (e = Cl(e.type, !0)), e;
    default:
      return "";
  }
}
function ms(e) {
  if (e == null) return null;
  if (typeof e == "function") return e.displayName || e.name || null;
  if (typeof e == "string") return e;
  switch (e) {
    case In:
      return "Fragment";
    case Fn:
      return "Portal";
    case ds:
      return "Profiler";
    case Pu:
      return "StrictMode";
    case hs:
      return "Suspense";
    case ps:
      return "SuspenseList";
  }
  if (typeof e == "object")
    switch (e.$$typeof) {
      case fd:
        return (e.displayName || "Context") + ".Consumer";
      case cd:
        return (e._context.displayName || "Context") + ".Provider";
      case Cu:
        var t = e.render;
        return (
          (e = e.displayName),
          e ||
            ((e = t.displayName || t.name || ""),
            (e = e !== "" ? "ForwardRef(" + e + ")" : "ForwardRef")),
          e
        );
      case Mu:
        return (
          (t = e.displayName || null), t !== null ? t : ms(e.type) || "Memo"
        );
      case Ot:
        (t = e._payload), (e = e._init);
        try {
          return ms(e(t));
        } catch {}
    }
  return null;
}
function A0(e) {
  var t = e.type;
  switch (e.tag) {
    case 24:
      return "Cache";
    case 9:
      return (t.displayName || "Context") + ".Consumer";
    case 10:
      return (t._context.displayName || "Context") + ".Provider";
    case 18:
      return "DehydratedFragment";
    case 11:
      return (
        (e = t.render),
        (e = e.displayName || e.name || ""),
        t.displayName || (e !== "" ? "ForwardRef(" + e + ")" : "ForwardRef")
      );
    case 7:
      return "Fragment";
    case 5:
      return t;
    case 4:
      return "Portal";
    case 3:
      return "Root";
    case 6:
      return "Text";
    case 16:
      return ms(t);
    case 8:
      return t === Pu ? "StrictMode" : "Mode";
    case 22:
      return "Offscreen";
    case 12:
      return "Profiler";
    case 21:
      return "Scope";
    case 13:
      return "Suspense";
    case 19:
      return "SuspenseList";
    case 25:
      return "TracingMarker";
    case 1:
    case 0:
    case 17:
    case 2:
    case 14:
    case 15:
      if (typeof t == "function") return t.displayName || t.name || null;
      if (typeof t == "string") return t;
  }
  return null;
}
function Zt(e) {
  switch (typeof e) {
    case "boolean":
    case "number":
    case "string":
    case "undefined":
      return e;
    case "object":
      return e;
    default:
      return "";
  }
}
function hd(e) {
  var t = e.type;
  return (
    (e = e.nodeName) &&
    e.toLowerCase() === "input" &&
    (t === "checkbox" || t === "radio")
  );
}
function D0(e) {
  var t = hd(e) ? "checked" : "value",
    n = Object.getOwnPropertyDescriptor(e.constructor.prototype, t),
    r = "" + e[t];
  if (
    !e.hasOwnProperty(t) &&
    typeof n < "u" &&
    typeof n.get == "function" &&
    typeof n.set == "function"
  ) {
    var i = n.get,
      o = n.set;
    return (
      Object.defineProperty(e, t, {
        configurable: !0,
        get: function () {
          return i.call(this);
        },
        set: function (l) {
          (r = "" + l), o.call(this, l);
        },
      }),
      Object.defineProperty(e, t, { enumerable: n.enumerable }),
      {
        getValue: function () {
          return r;
        },
        setValue: function (l) {
          r = "" + l;
        },
        stopTracking: function () {
          (e._valueTracker = null), delete e[t];
        },
      }
    );
  }
}
function Fi(e) {
  e._valueTracker || (e._valueTracker = D0(e));
}
function pd(e) {
  if (!e) return !1;
  var t = e._valueTracker;
  if (!t) return !0;
  var n = t.getValue(),
    r = "";
  return (
    e && (r = hd(e) ? (e.checked ? "true" : "false") : e.value),
    (e = r),
    e !== n ? (t.setValue(e), !0) : !1
  );
}
function go(e) {
  if (((e = e || (typeof document < "u" ? document : void 0)), typeof e > "u"))
    return null;
  try {
    return e.activeElement || e.body;
  } catch {
    return e.body;
  }
}
function ys(e, t) {
  var n = t.checked;
  return ie({}, t, {
    defaultChecked: void 0,
    defaultValue: void 0,
    value: void 0,
    checked: n ?? e._wrapperState.initialChecked,
  });
}
function Wa(e, t) {
  var n = t.defaultValue == null ? "" : t.defaultValue,
    r = t.checked != null ? t.checked : t.defaultChecked;
  (n = Zt(t.value != null ? t.value : n)),
    (e._wrapperState = {
      initialChecked: r,
      initialValue: n,
      controlled:
        t.type === "checkbox" || t.type === "radio"
          ? t.checked != null
          : t.value != null,
    });
}
function md(e, t) {
  (t = t.checked), t != null && Eu(e, "checked", t, !1);
}
function gs(e, t) {
  md(e, t);
  var n = Zt(t.value),
    r = t.type;
  if (n != null)
    r === "number"
      ? ((n === 0 && e.value === "") || e.value != n) && (e.value = "" + n)
      : e.value !== "" + n && (e.value = "" + n);
  else if (r === "submit" || r === "reset") {
    e.removeAttribute("value");
    return;
  }
  t.hasOwnProperty("value")
    ? xs(e, t.type, n)
    : t.hasOwnProperty("defaultValue") && xs(e, t.type, Zt(t.defaultValue)),
    t.checked == null &&
      t.defaultChecked != null &&
      (e.defaultChecked = !!t.defaultChecked);
}
function Va(e, t, n) {
  if (t.hasOwnProperty("value") || t.hasOwnProperty("defaultValue")) {
    var r = t.type;
    if (
      !(
        (r !== "submit" && r !== "reset") ||
        (t.value !== void 0 && t.value !== null)
      )
    )
      return;
    (t = "" + e._wrapperState.initialValue),
      n || t === e.value || (e.value = t),
      (e.defaultValue = t);
  }
  (n = e.name),
    n !== "" && (e.name = ""),
    (e.defaultChecked = !!e._wrapperState.initialChecked),
    n !== "" && (e.name = n);
}
function xs(e, t, n) {
  (t !== "number" || go(e.ownerDocument) !== e) &&
    (n == null
      ? (e.defaultValue = "" + e._wrapperState.initialValue)
      : e.defaultValue !== "" + n && (e.defaultValue = "" + n));
}
var Ir = Array.isArray;
function qn(e, t, n, r) {
  if (((e = e.options), t)) {
    t = {};
    for (var i = 0; i < n.length; i++) t["$" + n[i]] = !0;
    for (n = 0; n < e.length; n++)
      (i = t.hasOwnProperty("$" + e[n].value)),
        e[n].selected !== i && (e[n].selected = i),
        i && r && (e[n].defaultSelected = !0);
  } else {
    for (n = "" + Zt(n), t = null, i = 0; i < e.length; i++) {
      if (e[i].value === n) {
        (e[i].selected = !0), r && (e[i].defaultSelected = !0);
        return;
      }
      t !== null || e[i].disabled || (t = e[i]);
    }
    t !== null && (t.selected = !0);
  }
}
function vs(e, t) {
  if (t.dangerouslySetInnerHTML != null) throw Error(C(91));
  return ie({}, t, {
    value: void 0,
    defaultValue: void 0,
    children: "" + e._wrapperState.initialValue,
  });
}
function Ba(e, t) {
  var n = t.value;
  if (n == null) {
    if (((n = t.children), (t = t.defaultValue), n != null)) {
      if (t != null) throw Error(C(92));
      if (Ir(n)) {
        if (1 < n.length) throw Error(C(93));
        n = n[0];
      }
      t = n;
    }
    t == null && (t = ""), (n = t);
  }
  e._wrapperState = { initialValue: Zt(n) };
}
function yd(e, t) {
  var n = Zt(t.value),
    r = Zt(t.defaultValue);
  n != null &&
    ((n = "" + n),
    n !== e.value && (e.value = n),
    t.defaultValue == null && e.defaultValue !== n && (e.defaultValue = n)),
    r != null && (e.defaultValue = "" + r);
}
function Ya(e) {
  var t = e.textContent;
  t === e._wrapperState.initialValue && t !== "" && t !== null && (e.value = t);
}
function gd(e) {
  switch (e) {
    case "svg":
      return "http://www.w3.org/2000/svg";
    case "math":
      return "http://www.w3.org/1998/Math/MathML";
    default:
      return "http://www.w3.org/1999/xhtml";
  }
}
function ws(e, t) {
  return e == null || e === "http://www.w3.org/1999/xhtml"
    ? gd(t)
    : e === "http://www.w3.org/2000/svg" && t === "foreignObject"
    ? "http://www.w3.org/1999/xhtml"
    : e;
}
var Ii,
  xd = (function (e) {
    return typeof MSApp < "u" && MSApp.execUnsafeLocalFunction
      ? function (t, n, r, i) {
          MSApp.execUnsafeLocalFunction(function () {
            return e(t, n, r, i);
          });
        }
      : e;
  })(function (e, t) {
    if (e.namespaceURI !== "http://www.w3.org/2000/svg" || "innerHTML" in e)
      e.innerHTML = t;
    else {
      for (
        Ii = Ii || document.createElement("div"),
          Ii.innerHTML = "<svg>" + t.valueOf().toString() + "</svg>",
          t = Ii.firstChild;
        e.firstChild;

      )
        e.removeChild(e.firstChild);
      for (; t.firstChild; ) e.appendChild(t.firstChild);
    }
  });
function ri(e, t) {
  if (t) {
    var n = e.firstChild;
    if (n && n === e.lastChild && n.nodeType === 3) {
      n.nodeValue = t;
      return;
    }
  }
  e.textContent = t;
}
var Qr = {
    animationIterationCount: !0,
    aspectRatio: !0,
    borderImageOutset: !0,
    borderImageSlice: !0,
    borderImageWidth: !0,
    boxFlex: !0,
    boxFlexGroup: !0,
    boxOrdinalGroup: !0,
    columnCount: !0,
    columns: !0,
    flex: !0,
    flexGrow: !0,
    flexPositive: !0,
    flexShrink: !0,
    flexNegative: !0,
    flexOrder: !0,
    gridArea: !0,
    gridRow: !0,
    gridRowEnd: !0,
    gridRowSpan: !0,
    gridRowStart: !0,
    gridColumn: !0,
    gridColumnEnd: !0,
    gridColumnSpan: !0,
    gridColumnStart: !0,
    fontWeight: !0,
    lineClamp: !0,
    lineHeight: !0,
    opacity: !0,
    order: !0,
    orphans: !0,
    tabSize: !0,
    widows: !0,
    zIndex: !0,
    zoom: !0,
    fillOpacity: !0,
    floodOpacity: !0,
    stopOpacity: !0,
    strokeDasharray: !0,
    strokeDashoffset: !0,
    strokeMiterlimit: !0,
    strokeOpacity: !0,
    strokeWidth: !0,
  },
  z0 = ["Webkit", "ms", "Moz", "O"];
Object.keys(Qr).forEach(function (e) {
  z0.forEach(function (t) {
    (t = t + e.charAt(0).toUpperCase() + e.substring(1)), (Qr[t] = Qr[e]);
  });
});
function vd(e, t, n) {
  return t == null || typeof t == "boolean" || t === ""
    ? ""
    : n || typeof t != "number" || t === 0 || (Qr.hasOwnProperty(e) && Qr[e])
    ? ("" + t).trim()
    : t + "px";
}
function wd(e, t) {
  e = e.style;
  for (var n in t)
    if (t.hasOwnProperty(n)) {
      var r = n.indexOf("--") === 0,
        i = vd(n, t[n], r);
      n === "float" && (n = "cssFloat"), r ? e.setProperty(n, i) : (e[n] = i);
    }
}
var O0 = ie(
  { menuitem: !0 },
  {
    area: !0,
    base: !0,
    br: !0,
    col: !0,
    embed: !0,
    hr: !0,
    img: !0,
    input: !0,
    keygen: !0,
    link: !0,
    meta: !0,
    param: !0,
    source: !0,
    track: !0,
    wbr: !0,
  }
);
function Ss(e, t) {
  if (t) {
    if (O0[e] && (t.children != null || t.dangerouslySetInnerHTML != null))
      throw Error(C(137, e));
    if (t.dangerouslySetInnerHTML != null) {
      if (t.children != null) throw Error(C(60));
      if (
        typeof t.dangerouslySetInnerHTML != "object" ||
        !("__html" in t.dangerouslySetInnerHTML)
      )
        throw Error(C(61));
    }
    if (t.style != null && typeof t.style != "object") throw Error(C(62));
  }
}
function ks(e, t) {
  if (e.indexOf("-") === -1) return typeof t.is == "string";
  switch (e) {
    case "annotation-xml":
    case "color-profile":
    case "font-face":
    case "font-face-src":
    case "font-face-uri":
    case "font-face-format":
    case "font-face-name":
    case "missing-glyph":
      return !1;
    default:
      return !0;
  }
}
var js = null;
function Tu(e) {
  return (
    (e = e.target || e.srcElement || window),
    e.correspondingUseElement && (e = e.correspondingUseElement),
    e.nodeType === 3 ? e.parentNode : e
  );
}
var Es = null,
  er = null,
  tr = null;
function Qa(e) {
  if ((e = _i(e))) {
    if (typeof Es != "function") throw Error(C(280));
    var t = e.stateNode;
    t && ((t = ol(t)), Es(e.stateNode, e.type, t));
  }
}
function Sd(e) {
  er ? (tr ? tr.push(e) : (tr = [e])) : (er = e);
}
function kd() {
  if (er) {
    var e = er,
      t = tr;
    if (((tr = er = null), Qa(e), t)) for (e = 0; e < t.length; e++) Qa(t[e]);
  }
}
function jd(e, t) {
  return e(t);
}
function Ed() {}
var Ml = !1;
function Pd(e, t, n) {
  if (Ml) return e(t, n);
  Ml = !0;
  try {
    return jd(e, t, n);
  } finally {
    (Ml = !1), (er !== null || tr !== null) && (Ed(), kd());
  }
}
function ii(e, t) {
  var n = e.stateNode;
  if (n === null) return null;
  var r = ol(n);
  if (r === null) return null;
  n = r[t];
  e: switch (t) {
    case "onClick":
    case "onClickCapture":
    case "onDoubleClick":
    case "onDoubleClickCapture":
    case "onMouseDown":
    case "onMouseDownCapture":
    case "onMouseMove":
    case "onMouseMoveCapture":
    case "onMouseUp":
    case "onMouseUpCapture":
    case "onMouseEnter":
      (r = !r.disabled) ||
        ((e = e.type),
        (r = !(
          e === "button" ||
          e === "input" ||
          e === "select" ||
          e === "textarea"
        ))),
        (e = !r);
      break e;
    default:
      e = !1;
  }
  if (e) return null;
  if (n && typeof n != "function") throw Error(C(231, t, typeof n));
  return n;
}
var Ps = !1;
if (Tt)
  try {
    var Pr = {};
    Object.defineProperty(Pr, "passive", {
      get: function () {
        Ps = !0;
      },
    }),
      window.addEventListener("test", Pr, Pr),
      window.removeEventListener("test", Pr, Pr);
  } catch {
    Ps = !1;
  }
function R0(e, t, n, r, i, o, l, s, u) {
  var a = Array.prototype.slice.call(arguments, 3);
  try {
    t.apply(n, a);
  } catch (f) {
    this.onError(f);
  }
}
var Gr = !1,
  xo = null,
  vo = !1,
  Cs = null,
  F0 = {
    onError: function (e) {
      (Gr = !0), (xo = e);
    },
  };
function I0(e, t, n, r, i, o, l, s, u) {
  (Gr = !1), (xo = null), R0.apply(F0, arguments);
}
function U0(e, t, n, r, i, o, l, s, u) {
  if ((I0.apply(this, arguments), Gr)) {
    if (Gr) {
      var a = xo;
      (Gr = !1), (xo = null);
    } else throw Error(C(198));
    vo || ((vo = !0), (Cs = a));
  }
}
function _n(e) {
  var t = e,
    n = e;
  if (e.alternate) for (; t.return; ) t = t.return;
  else {
    e = t;
    do (t = e), t.flags & 4098 && (n = t.return), (e = t.return);
    while (e);
  }
  return t.tag === 3 ? n : null;
}
function Cd(e) {
  if (e.tag === 13) {
    var t = e.memoizedState;
    if (
      (t === null && ((e = e.alternate), e !== null && (t = e.memoizedState)),
      t !== null)
    )
      return t.dehydrated;
  }
  return null;
}
function Ga(e) {
  if (_n(e) !== e) throw Error(C(188));
}
function H0(e) {
  var t = e.alternate;
  if (!t) {
    if (((t = _n(e)), t === null)) throw Error(C(188));
    return t !== e ? null : e;
  }
  for (var n = e, r = t; ; ) {
    var i = n.return;
    if (i === null) break;
    var o = i.alternate;
    if (o === null) {
      if (((r = i.return), r !== null)) {
        n = r;
        continue;
      }
      break;
    }
    if (i.child === o.child) {
      for (o = i.child; o; ) {
        if (o === n) return Ga(i), e;
        if (o === r) return Ga(i), t;
        o = o.sibling;
      }
      throw Error(C(188));
    }
    if (n.return !== r.return) (n = i), (r = o);
    else {
      for (var l = !1, s = i.child; s; ) {
        if (s === n) {
          (l = !0), (n = i), (r = o);
          break;
        }
        if (s === r) {
          (l = !0), (r = i), (n = o);
          break;
        }
        s = s.sibling;
      }
      if (!l) {
        for (s = o.child; s; ) {
          if (s === n) {
            (l = !0), (n = o), (r = i);
            break;
          }
          if (s === r) {
            (l = !0), (r = o), (n = i);
            break;
          }
          s = s.sibling;
        }
        if (!l) throw Error(C(189));
      }
    }
    if (n.alternate !== r) throw Error(C(190));
  }
  if (n.tag !== 3) throw Error(C(188));
  return n.stateNode.current === n ? e : t;
}
function Md(e) {
  return (e = H0(e)), e !== null ? Td(e) : null;
}
function Td(e) {
  if (e.tag === 5 || e.tag === 6) return e;
  for (e = e.child; e !== null; ) {
    var t = Td(e);
    if (t !== null) return t;
    e = e.sibling;
  }
  return null;
}
var _d = We.unstable_scheduleCallback,
  ba = We.unstable_cancelCallback,
  W0 = We.unstable_shouldYield,
  V0 = We.unstable_requestPaint,
  se = We.unstable_now,
  B0 = We.unstable_getCurrentPriorityLevel,
  _u = We.unstable_ImmediatePriority,
  Ld = We.unstable_UserBlockingPriority,
  wo = We.unstable_NormalPriority,
  Y0 = We.unstable_LowPriority,
  Nd = We.unstable_IdlePriority,
  tl = null,
  yt = null;
function Q0(e) {
  if (yt && typeof yt.onCommitFiberRoot == "function")
    try {
      yt.onCommitFiberRoot(tl, e, void 0, (e.current.flags & 128) === 128);
    } catch {}
}
var at = Math.clz32 ? Math.clz32 : X0,
  G0 = Math.log,
  b0 = Math.LN2;
function X0(e) {
  return (e >>>= 0), e === 0 ? 32 : (31 - ((G0(e) / b0) | 0)) | 0;
}
var Ui = 64,
  Hi = 4194304;
function Ur(e) {
  switch (e & -e) {
    case 1:
      return 1;
    case 2:
      return 2;
    case 4:
      return 4;
    case 8:
      return 8;
    case 16:
      return 16;
    case 32:
      return 32;
    case 64:
    case 128:
    case 256:
    case 512:
    case 1024:
    case 2048:
    case 4096:
    case 8192:
    case 16384:
    case 32768:
    case 65536:
    case 131072:
    case 262144:
    case 524288:
    case 1048576:
    case 2097152:
      return e & 4194240;
    case 4194304:
    case 8388608:
    case 16777216:
    case 33554432:
    case 67108864:
      return e & 130023424;
    case 134217728:
      return 134217728;
    case 268435456:
      return 268435456;
    case 536870912:
      return 536870912;
    case 1073741824:
      return 1073741824;
    default:
      return e;
  }
}
function So(e, t) {
  var n = e.pendingLanes;
  if (n === 0) return 0;
  var r = 0,
    i = e.suspendedLanes,
    o = e.pingedLanes,
    l = n & 268435455;
  if (l !== 0) {
    var s = l & ~i;
    s !== 0 ? (r = Ur(s)) : ((o &= l), o !== 0 && (r = Ur(o)));
  } else (l = n & ~i), l !== 0 ? (r = Ur(l)) : o !== 0 && (r = Ur(o));
  if (r === 0) return 0;
  if (
    t !== 0 &&
    t !== r &&
    !(t & i) &&
    ((i = r & -r), (o = t & -t), i >= o || (i === 16 && (o & 4194240) !== 0))
  )
    return t;
  if ((r & 4 && (r |= n & 16), (t = e.entangledLanes), t !== 0))
    for (e = e.entanglements, t &= r; 0 < t; )
      (n = 31 - at(t)), (i = 1 << n), (r |= e[n]), (t &= ~i);
  return r;
}
function K0(e, t) {
  switch (e) {
    case 1:
    case 2:
    case 4:
      return t + 250;
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
    case 256:
    case 512:
    case 1024:
    case 2048:
    case 4096:
    case 8192:
    case 16384:
    case 32768:
    case 65536:
    case 131072:
    case 262144:
    case 524288:
    case 1048576:
    case 2097152:
      return t + 5e3;
    case 4194304:
    case 8388608:
    case 16777216:
    case 33554432:
    case 67108864:
      return -1;
    case 134217728:
    case 268435456:
    case 536870912:
    case 1073741824:
      return -1;
    default:
      return -1;
  }
}
function Z0(e, t) {
  for (
    var n = e.suspendedLanes,
      r = e.pingedLanes,
      i = e.expirationTimes,
      o = e.pendingLanes;
    0 < o;

  ) {
    var l = 31 - at(o),
      s = 1 << l,
      u = i[l];
    u === -1
      ? (!(s & n) || s & r) && (i[l] = K0(s, t))
      : u <= t && (e.expiredLanes |= s),
      (o &= ~s);
  }
}
function Ms(e) {
  return (
    (e = e.pendingLanes & -1073741825),
    e !== 0 ? e : e & 1073741824 ? 1073741824 : 0
  );
}
function $d() {
  var e = Ui;
  return (Ui <<= 1), !(Ui & 4194240) && (Ui = 64), e;
}
function Tl(e) {
  for (var t = [], n = 0; 31 > n; n++) t.push(e);
  return t;
}
function Mi(e, t, n) {
  (e.pendingLanes |= t),
    t !== 536870912 && ((e.suspendedLanes = 0), (e.pingedLanes = 0)),
    (e = e.eventTimes),
    (t = 31 - at(t)),
    (e[t] = n);
}
function J0(e, t) {
  var n = e.pendingLanes & ~t;
  (e.pendingLanes = t),
    (e.suspendedLanes = 0),
    (e.pingedLanes = 0),
    (e.expiredLanes &= t),
    (e.mutableReadLanes &= t),
    (e.entangledLanes &= t),
    (t = e.entanglements);
  var r = e.eventTimes;
  for (e = e.expirationTimes; 0 < n; ) {
    var i = 31 - at(n),
      o = 1 << i;
    (t[i] = 0), (r[i] = -1), (e[i] = -1), (n &= ~o);
  }
}
function Lu(e, t) {
  var n = (e.entangledLanes |= t);
  for (e = e.entanglements; n; ) {
    var r = 31 - at(n),
      i = 1 << r;
    (i & t) | (e[r] & t) && (e[r] |= t), (n &= ~i);
  }
}
var G = 0;
function Ad(e) {
  return (e &= -e), 1 < e ? (4 < e ? (e & 268435455 ? 16 : 536870912) : 4) : 1;
}
var Dd,
  Nu,
  zd,
  Od,
  Rd,
  Ts = !1,
  Wi = [],
  Vt = null,
  Bt = null,
  Yt = null,
  oi = new Map(),
  li = new Map(),
  It = [],
  q0 =
    "mousedown mouseup touchcancel touchend touchstart auxclick dblclick pointercancel pointerdown pointerup dragend dragstart drop compositionend compositionstart keydown keypress keyup input textInput copy cut paste click change contextmenu reset submit".split(
      " "
    );
function Xa(e, t) {
  switch (e) {
    case "focusin":
    case "focusout":
      Vt = null;
      break;
    case "dragenter":
    case "dragleave":
      Bt = null;
      break;
    case "mouseover":
    case "mouseout":
      Yt = null;
      break;
    case "pointerover":
    case "pointerout":
      oi.delete(t.pointerId);
      break;
    case "gotpointercapture":
    case "lostpointercapture":
      li.delete(t.pointerId);
  }
}
function Cr(e, t, n, r, i, o) {
  return e === null || e.nativeEvent !== o
    ? ((e = {
        blockedOn: t,
        domEventName: n,
        eventSystemFlags: r,
        nativeEvent: o,
        targetContainers: [i],
      }),
      t !== null && ((t = _i(t)), t !== null && Nu(t)),
      e)
    : ((e.eventSystemFlags |= r),
      (t = e.targetContainers),
      i !== null && t.indexOf(i) === -1 && t.push(i),
      e);
}
function em(e, t, n, r, i) {
  switch (t) {
    case "focusin":
      return (Vt = Cr(Vt, e, t, n, r, i)), !0;
    case "dragenter":
      return (Bt = Cr(Bt, e, t, n, r, i)), !0;
    case "mouseover":
      return (Yt = Cr(Yt, e, t, n, r, i)), !0;
    case "pointerover":
      var o = i.pointerId;
      return oi.set(o, Cr(oi.get(o) || null, e, t, n, r, i)), !0;
    case "gotpointercapture":
      return (
        (o = i.pointerId), li.set(o, Cr(li.get(o) || null, e, t, n, r, i)), !0
      );
  }
  return !1;
}
function Fd(e) {
  var t = cn(e.target);
  if (t !== null) {
    var n = _n(t);
    if (n !== null) {
      if (((t = n.tag), t === 13)) {
        if (((t = Cd(n)), t !== null)) {
          (e.blockedOn = t),
            Rd(e.priority, function () {
              zd(n);
            });
          return;
        }
      } else if (t === 3 && n.stateNode.current.memoizedState.isDehydrated) {
        e.blockedOn = n.tag === 3 ? n.stateNode.containerInfo : null;
        return;
      }
    }
  }
  e.blockedOn = null;
}
function oo(e) {
  if (e.blockedOn !== null) return !1;
  for (var t = e.targetContainers; 0 < t.length; ) {
    var n = _s(e.domEventName, e.eventSystemFlags, t[0], e.nativeEvent);
    if (n === null) {
      n = e.nativeEvent;
      var r = new n.constructor(n.type, n);
      (js = r), n.target.dispatchEvent(r), (js = null);
    } else return (t = _i(n)), t !== null && Nu(t), (e.blockedOn = n), !1;
    t.shift();
  }
  return !0;
}
function Ka(e, t, n) {
  oo(e) && n.delete(t);
}
function tm() {
  (Ts = !1),
    Vt !== null && oo(Vt) && (Vt = null),
    Bt !== null && oo(Bt) && (Bt = null),
    Yt !== null && oo(Yt) && (Yt = null),
    oi.forEach(Ka),
    li.forEach(Ka);
}
function Mr(e, t) {
  e.blockedOn === t &&
    ((e.blockedOn = null),
    Ts ||
      ((Ts = !0),
      We.unstable_scheduleCallback(We.unstable_NormalPriority, tm)));
}
function si(e) {
  function t(i) {
    return Mr(i, e);
  }
  if (0 < Wi.length) {
    Mr(Wi[0], e);
    for (var n = 1; n < Wi.length; n++) {
      var r = Wi[n];
      r.blockedOn === e && (r.blockedOn = null);
    }
  }
  for (
    Vt !== null && Mr(Vt, e),
      Bt !== null && Mr(Bt, e),
      Yt !== null && Mr(Yt, e),
      oi.forEach(t),
      li.forEach(t),
      n = 0;
    n < It.length;
    n++
  )
    (r = It[n]), r.blockedOn === e && (r.blockedOn = null);
  for (; 0 < It.length && ((n = It[0]), n.blockedOn === null); )
    Fd(n), n.blockedOn === null && It.shift();
}
var nr = Dt.ReactCurrentBatchConfig,
  ko = !0;
function nm(e, t, n, r) {
  var i = G,
    o = nr.transition;
  nr.transition = null;
  try {
    (G = 1), $u(e, t, n, r);
  } finally {
    (G = i), (nr.transition = o);
  }
}
function rm(e, t, n, r) {
  var i = G,
    o = nr.transition;
  nr.transition = null;
  try {
    (G = 4), $u(e, t, n, r);
  } finally {
    (G = i), (nr.transition = o);
  }
}
function $u(e, t, n, r) {
  if (ko) {
    var i = _s(e, t, n, r);
    if (i === null) Fl(e, t, r, jo, n), Xa(e, r);
    else if (em(i, e, t, n, r)) r.stopPropagation();
    else if ((Xa(e, r), t & 4 && -1 < q0.indexOf(e))) {
      for (; i !== null; ) {
        var o = _i(i);
        if (
          (o !== null && Dd(o),
          (o = _s(e, t, n, r)),
          o === null && Fl(e, t, r, jo, n),
          o === i)
        )
          break;
        i = o;
      }
      i !== null && r.stopPropagation();
    } else Fl(e, t, r, null, n);
  }
}
var jo = null;
function _s(e, t, n, r) {
  if (((jo = null), (e = Tu(r)), (e = cn(e)), e !== null))
    if (((t = _n(e)), t === null)) e = null;
    else if (((n = t.tag), n === 13)) {
      if (((e = Cd(t)), e !== null)) return e;
      e = null;
    } else if (n === 3) {
      if (t.stateNode.current.memoizedState.isDehydrated)
        return t.tag === 3 ? t.stateNode.containerInfo : null;
      e = null;
    } else t !== e && (e = null);
  return (jo = e), null;
}
function Id(e) {
  switch (e) {
    case "cancel":
    case "click":
    case "close":
    case "contextmenu":
    case "copy":
    case "cut":
    case "auxclick":
    case "dblclick":
    case "dragend":
    case "dragstart":
    case "drop":
    case "focusin":
    case "focusout":
    case "input":
    case "invalid":
    case "keydown":
    case "keypress":
    case "keyup":
    case "mousedown":
    case "mouseup":
    case "paste":
    case "pause":
    case "play":
    case "pointercancel":
    case "pointerdown":
    case "pointerup":
    case "ratechange":
    case "reset":
    case "resize":
    case "seeked":
    case "submit":
    case "touchcancel":
    case "touchend":
    case "touchstart":
    case "volumechange":
    case "change":
    case "selectionchange":
    case "textInput":
    case "compositionstart":
    case "compositionend":
    case "compositionupdate":
    case "beforeblur":
    case "afterblur":
    case "beforeinput":
    case "blur":
    case "fullscreenchange":
    case "focus":
    case "hashchange":
    case "popstate":
    case "select":
    case "selectstart":
      return 1;
    case "drag":
    case "dragenter":
    case "dragexit":
    case "dragleave":
    case "dragover":
    case "mousemove":
    case "mouseout":
    case "mouseover":
    case "pointermove":
    case "pointerout":
    case "pointerover":
    case "scroll":
    case "toggle":
    case "touchmove":
    case "wheel":
    case "mouseenter":
    case "mouseleave":
    case "pointerenter":
    case "pointerleave":
      return 4;
    case "message":
      switch (B0()) {
        case _u:
          return 1;
        case Ld:
          return 4;
        case wo:
        case Y0:
          return 16;
        case Nd:
          return 536870912;
        default:
          return 16;
      }
    default:
      return 16;
  }
}
var Ht = null,
  Au = null,
  lo = null;
function Ud() {
  if (lo) return lo;
  var e,
    t = Au,
    n = t.length,
    r,
    i = "value" in Ht ? Ht.value : Ht.textContent,
    o = i.length;
  for (e = 0; e < n && t[e] === i[e]; e++);
  var l = n - e;
  for (r = 1; r <= l && t[n - r] === i[o - r]; r++);
  return (lo = i.slice(e, 1 < r ? 1 - r : void 0));
}
function so(e) {
  var t = e.keyCode;
  return (
    "charCode" in e
      ? ((e = e.charCode), e === 0 && t === 13 && (e = 13))
      : (e = t),
    e === 10 && (e = 13),
    32 <= e || e === 13 ? e : 0
  );
}
function Vi() {
  return !0;
}
function Za() {
  return !1;
}
function Qe(e) {
  function t(n, r, i, o, l) {
    (this._reactName = n),
      (this._targetInst = i),
      (this.type = r),
      (this.nativeEvent = o),
      (this.target = l),
      (this.currentTarget = null);
    for (var s in e)
      e.hasOwnProperty(s) && ((n = e[s]), (this[s] = n ? n(o) : o[s]));
    return (
      (this.isDefaultPrevented = (
        o.defaultPrevented != null ? o.defaultPrevented : o.returnValue === !1
      )
        ? Vi
        : Za),
      (this.isPropagationStopped = Za),
      this
    );
  }
  return (
    ie(t.prototype, {
      preventDefault: function () {
        this.defaultPrevented = !0;
        var n = this.nativeEvent;
        n &&
          (n.preventDefault
            ? n.preventDefault()
            : typeof n.returnValue != "unknown" && (n.returnValue = !1),
          (this.isDefaultPrevented = Vi));
      },
      stopPropagation: function () {
        var n = this.nativeEvent;
        n &&
          (n.stopPropagation
            ? n.stopPropagation()
            : typeof n.cancelBubble != "unknown" && (n.cancelBubble = !0),
          (this.isPropagationStopped = Vi));
      },
      persist: function () {},
      isPersistent: Vi,
    }),
    t
  );
}
var xr = {
    eventPhase: 0,
    bubbles: 0,
    cancelable: 0,
    timeStamp: function (e) {
      return e.timeStamp || Date.now();
    },
    defaultPrevented: 0,
    isTrusted: 0,
  },
  Du = Qe(xr),
  Ti = ie({}, xr, { view: 0, detail: 0 }),
  im = Qe(Ti),
  _l,
  Ll,
  Tr,
  nl = ie({}, Ti, {
    screenX: 0,
    screenY: 0,
    clientX: 0,
    clientY: 0,
    pageX: 0,
    pageY: 0,
    ctrlKey: 0,
    shiftKey: 0,
    altKey: 0,
    metaKey: 0,
    getModifierState: zu,
    button: 0,
    buttons: 0,
    relatedTarget: function (e) {
      return e.relatedTarget === void 0
        ? e.fromElement === e.srcElement
          ? e.toElement
          : e.fromElement
        : e.relatedTarget;
    },
    movementX: function (e) {
      return "movementX" in e
        ? e.movementX
        : (e !== Tr &&
            (Tr && e.type === "mousemove"
              ? ((_l = e.screenX - Tr.screenX), (Ll = e.screenY - Tr.screenY))
              : (Ll = _l = 0),
            (Tr = e)),
          _l);
    },
    movementY: function (e) {
      return "movementY" in e ? e.movementY : Ll;
    },
  }),
  Ja = Qe(nl),
  om = ie({}, nl, { dataTransfer: 0 }),
  lm = Qe(om),
  sm = ie({}, Ti, { relatedTarget: 0 }),
  Nl = Qe(sm),
  um = ie({}, xr, { animationName: 0, elapsedTime: 0, pseudoElement: 0 }),
  am = Qe(um),
  cm = ie({}, xr, {
    clipboardData: function (e) {
      return "clipboardData" in e ? e.clipboardData : window.clipboardData;
    },
  }),
  fm = Qe(cm),
  dm = ie({}, xr, { data: 0 }),
  qa = Qe(dm),
  hm = {
    Esc: "Escape",
    Spacebar: " ",
    Left: "ArrowLeft",
    Up: "ArrowUp",
    Right: "ArrowRight",
    Down: "ArrowDown",
    Del: "Delete",
    Win: "OS",
    Menu: "ContextMenu",
    Apps: "ContextMenu",
    Scroll: "ScrollLock",
    MozPrintableKey: "Unidentified",
  },
  pm = {
    8: "Backspace",
    9: "Tab",
    12: "Clear",
    13: "Enter",
    16: "Shift",
    17: "Control",
    18: "Alt",
    19: "Pause",
    20: "CapsLock",
    27: "Escape",
    32: " ",
    33: "PageUp",
    34: "PageDown",
    35: "End",
    36: "Home",
    37: "ArrowLeft",
    38: "ArrowUp",
    39: "ArrowRight",
    40: "ArrowDown",
    45: "Insert",
    46: "Delete",
    112: "F1",
    113: "F2",
    114: "F3",
    115: "F4",
    116: "F5",
    117: "F6",
    118: "F7",
    119: "F8",
    120: "F9",
    121: "F10",
    122: "F11",
    123: "F12",
    144: "NumLock",
    145: "ScrollLock",
    224: "Meta",
  },
  mm = {
    Alt: "altKey",
    Control: "ctrlKey",
    Meta: "metaKey",
    Shift: "shiftKey",
  };
function ym(e) {
  var t = this.nativeEvent;
  return t.getModifierState ? t.getModifierState(e) : (e = mm[e]) ? !!t[e] : !1;
}
function zu() {
  return ym;
}
var gm = ie({}, Ti, {
    key: function (e) {
      if (e.key) {
        var t = hm[e.key] || e.key;
        if (t !== "Unidentified") return t;
      }
      return e.type === "keypress"
        ? ((e = so(e)), e === 13 ? "Enter" : String.fromCharCode(e))
        : e.type === "keydown" || e.type === "keyup"
        ? pm[e.keyCode] || "Unidentified"
        : "";
    },
    code: 0,
    location: 0,
    ctrlKey: 0,
    shiftKey: 0,
    altKey: 0,
    metaKey: 0,
    repeat: 0,
    locale: 0,
    getModifierState: zu,
    charCode: function (e) {
      return e.type === "keypress" ? so(e) : 0;
    },
    keyCode: function (e) {
      return e.type === "keydown" || e.type === "keyup" ? e.keyCode : 0;
    },
    which: function (e) {
      return e.type === "keypress"
        ? so(e)
        : e.type === "keydown" || e.type === "keyup"
        ? e.keyCode
        : 0;
    },
  }),
  xm = Qe(gm),
  vm = ie({}, nl, {
    pointerId: 0,
    width: 0,
    height: 0,
    pressure: 0,
    tangentialPressure: 0,
    tiltX: 0,
    tiltY: 0,
    twist: 0,
    pointerType: 0,
    isPrimary: 0,
  }),
  ec = Qe(vm),
  wm = ie({}, Ti, {
    touches: 0,
    targetTouches: 0,
    changedTouches: 0,
    altKey: 0,
    metaKey: 0,
    ctrlKey: 0,
    shiftKey: 0,
    getModifierState: zu,
  }),
  Sm = Qe(wm),
  km = ie({}, xr, { propertyName: 0, elapsedTime: 0, pseudoElement: 0 }),
  jm = Qe(km),
  Em = ie({}, nl, {
    deltaX: function (e) {
      return "deltaX" in e ? e.deltaX : "wheelDeltaX" in e ? -e.wheelDeltaX : 0;
    },
    deltaY: function (e) {
      return "deltaY" in e
        ? e.deltaY
        : "wheelDeltaY" in e
        ? -e.wheelDeltaY
        : "wheelDelta" in e
        ? -e.wheelDelta
        : 0;
    },
    deltaZ: 0,
    deltaMode: 0,
  }),
  Pm = Qe(Em),
  Cm = [9, 13, 27, 32],
  Ou = Tt && "CompositionEvent" in window,
  br = null;
Tt && "documentMode" in document && (br = document.documentMode);
var Mm = Tt && "TextEvent" in window && !br,
  Hd = Tt && (!Ou || (br && 8 < br && 11 >= br)),
  tc = " ",
  nc = !1;
function Wd(e, t) {
  switch (e) {
    case "keyup":
      return Cm.indexOf(t.keyCode) !== -1;
    case "keydown":
      return t.keyCode !== 229;
    case "keypress":
    case "mousedown":
    case "focusout":
      return !0;
    default:
      return !1;
  }
}
function Vd(e) {
  return (e = e.detail), typeof e == "object" && "data" in e ? e.data : null;
}
var Un = !1;
function Tm(e, t) {
  switch (e) {
    case "compositionend":
      return Vd(t);
    case "keypress":
      return t.which !== 32 ? null : ((nc = !0), tc);
    case "textInput":
      return (e = t.data), e === tc && nc ? null : e;
    default:
      return null;
  }
}
function _m(e, t) {
  if (Un)
    return e === "compositionend" || (!Ou && Wd(e, t))
      ? ((e = Ud()), (lo = Au = Ht = null), (Un = !1), e)
      : null;
  switch (e) {
    case "paste":
      return null;
    case "keypress":
      if (!(t.ctrlKey || t.altKey || t.metaKey) || (t.ctrlKey && t.altKey)) {
        if (t.char && 1 < t.char.length) return t.char;
        if (t.which) return String.fromCharCode(t.which);
      }
      return null;
    case "compositionend":
      return Hd && t.locale !== "ko" ? null : t.data;
    default:
      return null;
  }
}
var Lm = {
  color: !0,
  date: !0,
  datetime: !0,
  "datetime-local": !0,
  email: !0,
  month: !0,
  number: !0,
  password: !0,
  range: !0,
  search: !0,
  tel: !0,
  text: !0,
  time: !0,
  url: !0,
  week: !0,
};
function rc(e) {
  var t = e && e.nodeName && e.nodeName.toLowerCase();
  return t === "input" ? !!Lm[e.type] : t === "textarea";
}
function Bd(e, t, n, r) {
  Sd(r),
    (t = Eo(t, "onChange")),
    0 < t.length &&
      ((n = new Du("onChange", "change", null, n, r)),
      e.push({ event: n, listeners: t }));
}
var Xr = null,
  ui = null;
function Nm(e) {
  th(e, 0);
}
function rl(e) {
  var t = Vn(e);
  if (pd(t)) return e;
}
function $m(e, t) {
  if (e === "change") return t;
}
var Yd = !1;
if (Tt) {
  var $l;
  if (Tt) {
    var Al = "oninput" in document;
    if (!Al) {
      var ic = document.createElement("div");
      ic.setAttribute("oninput", "return;"),
        (Al = typeof ic.oninput == "function");
    }
    $l = Al;
  } else $l = !1;
  Yd = $l && (!document.documentMode || 9 < document.documentMode);
}
function oc() {
  Xr && (Xr.detachEvent("onpropertychange", Qd), (ui = Xr = null));
}
function Qd(e) {
  if (e.propertyName === "value" && rl(ui)) {
    var t = [];
    Bd(t, ui, e, Tu(e)), Pd(Nm, t);
  }
}
function Am(e, t, n) {
  e === "focusin"
    ? (oc(), (Xr = t), (ui = n), Xr.attachEvent("onpropertychange", Qd))
    : e === "focusout" && oc();
}
function Dm(e) {
  if (e === "selectionchange" || e === "keyup" || e === "keydown")
    return rl(ui);
}
function zm(e, t) {
  if (e === "click") return rl(t);
}
function Om(e, t) {
  if (e === "input" || e === "change") return rl(t);
}
function Rm(e, t) {
  return (e === t && (e !== 0 || 1 / e === 1 / t)) || (e !== e && t !== t);
}
var ft = typeof Object.is == "function" ? Object.is : Rm;
function ai(e, t) {
  if (ft(e, t)) return !0;
  if (typeof e != "object" || e === null || typeof t != "object" || t === null)
    return !1;
  var n = Object.keys(e),
    r = Object.keys(t);
  if (n.length !== r.length) return !1;
  for (r = 0; r < n.length; r++) {
    var i = n[r];
    if (!fs.call(t, i) || !ft(e[i], t[i])) return !1;
  }
  return !0;
}
function lc(e) {
  for (; e && e.firstChild; ) e = e.firstChild;
  return e;
}
function sc(e, t) {
  var n = lc(e);
  e = 0;
  for (var r; n; ) {
    if (n.nodeType === 3) {
      if (((r = e + n.textContent.length), e <= t && r >= t))
        return { node: n, offset: t - e };
      e = r;
    }
    e: {
      for (; n; ) {
        if (n.nextSibling) {
          n = n.nextSibling;
          break e;
        }
        n = n.parentNode;
      }
      n = void 0;
    }
    n = lc(n);
  }
}
function Gd(e, t) {
  return e && t
    ? e === t
      ? !0
      : e && e.nodeType === 3
      ? !1
      : t && t.nodeType === 3
      ? Gd(e, t.parentNode)
      : "contains" in e
      ? e.contains(t)
      : e.compareDocumentPosition
      ? !!(e.compareDocumentPosition(t) & 16)
      : !1
    : !1;
}
function bd() {
  for (var e = window, t = go(); t instanceof e.HTMLIFrameElement; ) {
    try {
      var n = typeof t.contentWindow.location.href == "string";
    } catch {
      n = !1;
    }
    if (n) e = t.contentWindow;
    else break;
    t = go(e.document);
  }
  return t;
}
function Ru(e) {
  var t = e && e.nodeName && e.nodeName.toLowerCase();
  return (
    t &&
    ((t === "input" &&
      (e.type === "text" ||
        e.type === "search" ||
        e.type === "tel" ||
        e.type === "url" ||
        e.type === "password")) ||
      t === "textarea" ||
      e.contentEditable === "true")
  );
}
function Fm(e) {
  var t = bd(),
    n = e.focusedElem,
    r = e.selectionRange;
  if (
    t !== n &&
    n &&
    n.ownerDocument &&
    Gd(n.ownerDocument.documentElement, n)
  ) {
    if (r !== null && Ru(n)) {
      if (
        ((t = r.start),
        (e = r.end),
        e === void 0 && (e = t),
        "selectionStart" in n)
      )
        (n.selectionStart = t), (n.selectionEnd = Math.min(e, n.value.length));
      else if (
        ((e = ((t = n.ownerDocument || document) && t.defaultView) || window),
        e.getSelection)
      ) {
        e = e.getSelection();
        var i = n.textContent.length,
          o = Math.min(r.start, i);
        (r = r.end === void 0 ? o : Math.min(r.end, i)),
          !e.extend && o > r && ((i = r), (r = o), (o = i)),
          (i = sc(n, o));
        var l = sc(n, r);
        i &&
          l &&
          (e.rangeCount !== 1 ||
            e.anchorNode !== i.node ||
            e.anchorOffset !== i.offset ||
            e.focusNode !== l.node ||
            e.focusOffset !== l.offset) &&
          ((t = t.createRange()),
          t.setStart(i.node, i.offset),
          e.removeAllRanges(),
          o > r
            ? (e.addRange(t), e.extend(l.node, l.offset))
            : (t.setEnd(l.node, l.offset), e.addRange(t)));
      }
    }
    for (t = [], e = n; (e = e.parentNode); )
      e.nodeType === 1 &&
        t.push({ element: e, left: e.scrollLeft, top: e.scrollTop });
    for (typeof n.focus == "function" && n.focus(), n = 0; n < t.length; n++)
      (e = t[n]),
        (e.element.scrollLeft = e.left),
        (e.element.scrollTop = e.top);
  }
}
var Im = Tt && "documentMode" in document && 11 >= document.documentMode,
  Hn = null,
  Ls = null,
  Kr = null,
  Ns = !1;
function uc(e, t, n) {
  var r = n.window === n ? n.document : n.nodeType === 9 ? n : n.ownerDocument;
  Ns ||
    Hn == null ||
    Hn !== go(r) ||
    ((r = Hn),
    "selectionStart" in r && Ru(r)
      ? (r = { start: r.selectionStart, end: r.selectionEnd })
      : ((r = (
          (r.ownerDocument && r.ownerDocument.defaultView) ||
          window
        ).getSelection()),
        (r = {
          anchorNode: r.anchorNode,
          anchorOffset: r.anchorOffset,
          focusNode: r.focusNode,
          focusOffset: r.focusOffset,
        })),
    (Kr && ai(Kr, r)) ||
      ((Kr = r),
      (r = Eo(Ls, "onSelect")),
      0 < r.length &&
        ((t = new Du("onSelect", "select", null, t, n)),
        e.push({ event: t, listeners: r }),
        (t.target = Hn))));
}
function Bi(e, t) {
  var n = {};
  return (
    (n[e.toLowerCase()] = t.toLowerCase()),
    (n["Webkit" + e] = "webkit" + t),
    (n["Moz" + e] = "moz" + t),
    n
  );
}
var Wn = {
    animationend: Bi("Animation", "AnimationEnd"),
    animationiteration: Bi("Animation", "AnimationIteration"),
    animationstart: Bi("Animation", "AnimationStart"),
    transitionend: Bi("Transition", "TransitionEnd"),
  },
  Dl = {},
  Xd = {};
Tt &&
  ((Xd = document.createElement("div").style),
  "AnimationEvent" in window ||
    (delete Wn.animationend.animation,
    delete Wn.animationiteration.animation,
    delete Wn.animationstart.animation),
  "TransitionEvent" in window || delete Wn.transitionend.transition);
function il(e) {
  if (Dl[e]) return Dl[e];
  if (!Wn[e]) return e;
  var t = Wn[e],
    n;
  for (n in t) if (t.hasOwnProperty(n) && n in Xd) return (Dl[e] = t[n]);
  return e;
}
var Kd = il("animationend"),
  Zd = il("animationiteration"),
  Jd = il("animationstart"),
  qd = il("transitionend"),
  eh = new Map(),
  ac =
    "abort auxClick cancel canPlay canPlayThrough click close contextMenu copy cut drag dragEnd dragEnter dragExit dragLeave dragOver dragStart drop durationChange emptied encrypted ended error gotPointerCapture input invalid keyDown keyPress keyUp load loadedData loadedMetadata loadStart lostPointerCapture mouseDown mouseMove mouseOut mouseOver mouseUp paste pause play playing pointerCancel pointerDown pointerMove pointerOut pointerOver pointerUp progress rateChange reset resize seeked seeking stalled submit suspend timeUpdate touchCancel touchEnd touchStart volumeChange scroll toggle touchMove waiting wheel".split(
      " "
    );
function qt(e, t) {
  eh.set(e, t), Tn(t, [e]);
}
for (var zl = 0; zl < ac.length; zl++) {
  var Ol = ac[zl],
    Um = Ol.toLowerCase(),
    Hm = Ol[0].toUpperCase() + Ol.slice(1);
  qt(Um, "on" + Hm);
}
qt(Kd, "onAnimationEnd");
qt(Zd, "onAnimationIteration");
qt(Jd, "onAnimationStart");
qt("dblclick", "onDoubleClick");
qt("focusin", "onFocus");
qt("focusout", "onBlur");
qt(qd, "onTransitionEnd");
lr("onMouseEnter", ["mouseout", "mouseover"]);
lr("onMouseLeave", ["mouseout", "mouseover"]);
lr("onPointerEnter", ["pointerout", "pointerover"]);
lr("onPointerLeave", ["pointerout", "pointerover"]);
Tn(
  "onChange",
  "change click focusin focusout input keydown keyup selectionchange".split(" ")
);
Tn(
  "onSelect",
  "focusout contextmenu dragend focusin keydown keyup mousedown mouseup selectionchange".split(
    " "
  )
);
Tn("onBeforeInput", ["compositionend", "keypress", "textInput", "paste"]);
Tn(
  "onCompositionEnd",
  "compositionend focusout keydown keypress keyup mousedown".split(" ")
);
Tn(
  "onCompositionStart",
  "compositionstart focusout keydown keypress keyup mousedown".split(" ")
);
Tn(
  "onCompositionUpdate",
  "compositionupdate focusout keydown keypress keyup mousedown".split(" ")
);
var Hr =
    "abort canplay canplaythrough durationchange emptied encrypted ended error loadeddata loadedmetadata loadstart pause play playing progress ratechange resize seeked seeking stalled suspend timeupdate volumechange waiting".split(
      " "
    ),
  Wm = new Set("cancel close invalid load scroll toggle".split(" ").concat(Hr));
function cc(e, t, n) {
  var r = e.type || "unknown-event";
  (e.currentTarget = n), U0(r, t, void 0, e), (e.currentTarget = null);
}
function th(e, t) {
  t = (t & 4) !== 0;
  for (var n = 0; n < e.length; n++) {
    var r = e[n],
      i = r.event;
    r = r.listeners;
    e: {
      var o = void 0;
      if (t)
        for (var l = r.length - 1; 0 <= l; l--) {
          var s = r[l],
            u = s.instance,
            a = s.currentTarget;
          if (((s = s.listener), u !== o && i.isPropagationStopped())) break e;
          cc(i, s, a), (o = u);
        }
      else
        for (l = 0; l < r.length; l++) {
          if (
            ((s = r[l]),
            (u = s.instance),
            (a = s.currentTarget),
            (s = s.listener),
            u !== o && i.isPropagationStopped())
          )
            break e;
          cc(i, s, a), (o = u);
        }
    }
  }
  if (vo) throw ((e = Cs), (vo = !1), (Cs = null), e);
}
function q(e, t) {
  var n = t[Os];
  n === void 0 && (n = t[Os] = new Set());
  var r = e + "__bubble";
  n.has(r) || (nh(t, e, 2, !1), n.add(r));
}
function Rl(e, t, n) {
  var r = 0;
  t && (r |= 4), nh(n, e, r, t);
}
var Yi = "_reactListening" + Math.random().toString(36).slice(2);
function ci(e) {
  if (!e[Yi]) {
    (e[Yi] = !0),
      ad.forEach(function (n) {
        n !== "selectionchange" && (Wm.has(n) || Rl(n, !1, e), Rl(n, !0, e));
      });
    var t = e.nodeType === 9 ? e : e.ownerDocument;
    t === null || t[Yi] || ((t[Yi] = !0), Rl("selectionchange", !1, t));
  }
}
function nh(e, t, n, r) {
  switch (Id(t)) {
    case 1:
      var i = nm;
      break;
    case 4:
      i = rm;
      break;
    default:
      i = $u;
  }
  (n = i.bind(null, t, n, e)),
    (i = void 0),
    !Ps ||
      (t !== "touchstart" && t !== "touchmove" && t !== "wheel") ||
      (i = !0),
    r
      ? i !== void 0
        ? e.addEventListener(t, n, { capture: !0, passive: i })
        : e.addEventListener(t, n, !0)
      : i !== void 0
      ? e.addEventListener(t, n, { passive: i })
      : e.addEventListener(t, n, !1);
}
function Fl(e, t, n, r, i) {
  var o = r;
  if (!(t & 1) && !(t & 2) && r !== null)
    e: for (;;) {
      if (r === null) return;
      var l = r.tag;
      if (l === 3 || l === 4) {
        var s = r.stateNode.containerInfo;
        if (s === i || (s.nodeType === 8 && s.parentNode === i)) break;
        if (l === 4)
          for (l = r.return; l !== null; ) {
            var u = l.tag;
            if (
              (u === 3 || u === 4) &&
              ((u = l.stateNode.containerInfo),
              u === i || (u.nodeType === 8 && u.parentNode === i))
            )
              return;
            l = l.return;
          }
        for (; s !== null; ) {
          if (((l = cn(s)), l === null)) return;
          if (((u = l.tag), u === 5 || u === 6)) {
            r = o = l;
            continue e;
          }
          s = s.parentNode;
        }
      }
      r = r.return;
    }
  Pd(function () {
    var a = o,
      f = Tu(n),
      d = [];
    e: {
      var h = eh.get(e);
      if (h !== void 0) {
        var x = Du,
          g = e;
        switch (e) {
          case "keypress":
            if (so(n) === 0) break e;
          case "keydown":
          case "keyup":
            x = xm;
            break;
          case "focusin":
            (g = "focus"), (x = Nl);
            break;
          case "focusout":
            (g = "blur"), (x = Nl);
            break;
          case "beforeblur":
          case "afterblur":
            x = Nl;
            break;
          case "click":
            if (n.button === 2) break e;
          case "auxclick":
          case "dblclick":
          case "mousedown":
          case "mousemove":
          case "mouseup":
          case "mouseout":
          case "mouseover":
          case "contextmenu":
            x = Ja;
            break;
          case "drag":
          case "dragend":
          case "dragenter":
          case "dragexit":
          case "dragleave":
          case "dragover":
          case "dragstart":
          case "drop":
            x = lm;
            break;
          case "touchcancel":
          case "touchend":
          case "touchmove":
          case "touchstart":
            x = Sm;
            break;
          case Kd:
          case Zd:
          case Jd:
            x = am;
            break;
          case qd:
            x = jm;
            break;
          case "scroll":
            x = im;
            break;
          case "wheel":
            x = Pm;
            break;
          case "copy":
          case "cut":
          case "paste":
            x = fm;
            break;
          case "gotpointercapture":
          case "lostpointercapture":
          case "pointercancel":
          case "pointerdown":
          case "pointermove":
          case "pointerout":
          case "pointerover":
          case "pointerup":
            x = ec;
        }
        var v = (t & 4) !== 0,
          j = !v && e === "scroll",
          m = v ? (h !== null ? h + "Capture" : null) : h;
        v = [];
        for (var p = a, y; p !== null; ) {
          y = p;
          var w = y.stateNode;
          if (
            (y.tag === 5 &&
              w !== null &&
              ((y = w),
              m !== null && ((w = ii(p, m)), w != null && v.push(fi(p, w, y)))),
            j)
          )
            break;
          p = p.return;
        }
        0 < v.length &&
          ((h = new x(h, g, null, n, f)), d.push({ event: h, listeners: v }));
      }
    }
    if (!(t & 7)) {
      e: {
        if (
          ((h = e === "mouseover" || e === "pointerover"),
          (x = e === "mouseout" || e === "pointerout"),
          h &&
            n !== js &&
            (g = n.relatedTarget || n.fromElement) &&
            (cn(g) || g[_t]))
        )
          break e;
        if (
          (x || h) &&
          ((h =
            f.window === f
              ? f
              : (h = f.ownerDocument)
              ? h.defaultView || h.parentWindow
              : window),
          x
            ? ((g = n.relatedTarget || n.toElement),
              (x = a),
              (g = g ? cn(g) : null),
              g !== null &&
                ((j = _n(g)), g !== j || (g.tag !== 5 && g.tag !== 6)) &&
                (g = null))
            : ((x = null), (g = a)),
          x !== g)
        ) {
          if (
            ((v = Ja),
            (w = "onMouseLeave"),
            (m = "onMouseEnter"),
            (p = "mouse"),
            (e === "pointerout" || e === "pointerover") &&
              ((v = ec),
              (w = "onPointerLeave"),
              (m = "onPointerEnter"),
              (p = "pointer")),
            (j = x == null ? h : Vn(x)),
            (y = g == null ? h : Vn(g)),
            (h = new v(w, p + "leave", x, n, f)),
            (h.target = j),
            (h.relatedTarget = y),
            (w = null),
            cn(f) === a &&
              ((v = new v(m, p + "enter", g, n, f)),
              (v.target = y),
              (v.relatedTarget = j),
              (w = v)),
            (j = w),
            x && g)
          )
            t: {
              for (v = x, m = g, p = 0, y = v; y; y = An(y)) p++;
              for (y = 0, w = m; w; w = An(w)) y++;
              for (; 0 < p - y; ) (v = An(v)), p--;
              for (; 0 < y - p; ) (m = An(m)), y--;
              for (; p--; ) {
                if (v === m || (m !== null && v === m.alternate)) break t;
                (v = An(v)), (m = An(m));
              }
              v = null;
            }
          else v = null;
          x !== null && fc(d, h, x, v, !1),
            g !== null && j !== null && fc(d, j, g, v, !0);
        }
      }
      e: {
        if (
          ((h = a ? Vn(a) : window),
          (x = h.nodeName && h.nodeName.toLowerCase()),
          x === "select" || (x === "input" && h.type === "file"))
        )
          var S = $m;
        else if (rc(h))
          if (Yd) S = Om;
          else {
            S = Dm;
            var k = Am;
          }
        else
          (x = h.nodeName) &&
            x.toLowerCase() === "input" &&
            (h.type === "checkbox" || h.type === "radio") &&
            (S = zm);
        if (S && (S = S(e, a))) {
          Bd(d, S, n, f);
          break e;
        }
        k && k(e, h, a),
          e === "focusout" &&
            (k = h._wrapperState) &&
            k.controlled &&
            h.type === "number" &&
            xs(h, "number", h.value);
      }
      switch (((k = a ? Vn(a) : window), e)) {
        case "focusin":
          (rc(k) || k.contentEditable === "true") &&
            ((Hn = k), (Ls = a), (Kr = null));
          break;
        case "focusout":
          Kr = Ls = Hn = null;
          break;
        case "mousedown":
          Ns = !0;
          break;
        case "contextmenu":
        case "mouseup":
        case "dragend":
          (Ns = !1), uc(d, n, f);
          break;
        case "selectionchange":
          if (Im) break;
        case "keydown":
        case "keyup":
          uc(d, n, f);
      }
      var E;
      if (Ou)
        e: {
          switch (e) {
            case "compositionstart":
              var P = "onCompositionStart";
              break e;
            case "compositionend":
              P = "onCompositionEnd";
              break e;
            case "compositionupdate":
              P = "onCompositionUpdate";
              break e;
          }
          P = void 0;
        }
      else
        Un
          ? Wd(e, n) && (P = "onCompositionEnd")
          : e === "keydown" && n.keyCode === 229 && (P = "onCompositionStart");
      P &&
        (Hd &&
          n.locale !== "ko" &&
          (Un || P !== "onCompositionStart"
            ? P === "onCompositionEnd" && Un && (E = Ud())
            : ((Ht = f),
              (Au = "value" in Ht ? Ht.value : Ht.textContent),
              (Un = !0))),
        (k = Eo(a, P)),
        0 < k.length &&
          ((P = new qa(P, e, null, n, f)),
          d.push({ event: P, listeners: k }),
          E ? (P.data = E) : ((E = Vd(n)), E !== null && (P.data = E)))),
        (E = Mm ? Tm(e, n) : _m(e, n)) &&
          ((a = Eo(a, "onBeforeInput")),
          0 < a.length &&
            ((f = new qa("onBeforeInput", "beforeinput", null, n, f)),
            d.push({ event: f, listeners: a }),
            (f.data = E)));
    }
    th(d, t);
  });
}
function fi(e, t, n) {
  return { instance: e, listener: t, currentTarget: n };
}
function Eo(e, t) {
  for (var n = t + "Capture", r = []; e !== null; ) {
    var i = e,
      o = i.stateNode;
    i.tag === 5 &&
      o !== null &&
      ((i = o),
      (o = ii(e, n)),
      o != null && r.unshift(fi(e, o, i)),
      (o = ii(e, t)),
      o != null && r.push(fi(e, o, i))),
      (e = e.return);
  }
  return r;
}
function An(e) {
  if (e === null) return null;
  do e = e.return;
  while (e && e.tag !== 5);
  return e || null;
}
function fc(e, t, n, r, i) {
  for (var o = t._reactName, l = []; n !== null && n !== r; ) {
    var s = n,
      u = s.alternate,
      a = s.stateNode;
    if (u !== null && u === r) break;
    s.tag === 5 &&
      a !== null &&
      ((s = a),
      i
        ? ((u = ii(n, o)), u != null && l.unshift(fi(n, u, s)))
        : i || ((u = ii(n, o)), u != null && l.push(fi(n, u, s)))),
      (n = n.return);
  }
  l.length !== 0 && e.push({ event: t, listeners: l });
}
var Vm = /\r\n?/g,
  Bm = /\u0000|\uFFFD/g;
function dc(e) {
  return (typeof e == "string" ? e : "" + e)
    .replace(
      Vm,
      `
`
    )
    .replace(Bm, "");
}
function Qi(e, t, n) {
  if (((t = dc(t)), dc(e) !== t && n)) throw Error(C(425));
}
function Po() {}
var $s = null,
  As = null;
function Ds(e, t) {
  return (
    e === "textarea" ||
    e === "noscript" ||
    typeof t.children == "string" ||
    typeof t.children == "number" ||
    (typeof t.dangerouslySetInnerHTML == "object" &&
      t.dangerouslySetInnerHTML !== null &&
      t.dangerouslySetInnerHTML.__html != null)
  );
}
var zs = typeof setTimeout == "function" ? setTimeout : void 0,
  Ym = typeof clearTimeout == "function" ? clearTimeout : void 0,
  hc = typeof Promise == "function" ? Promise : void 0,
  Qm =
    typeof queueMicrotask == "function"
      ? queueMicrotask
      : typeof hc < "u"
      ? function (e) {
          return hc.resolve(null).then(e).catch(Gm);
        }
      : zs;
function Gm(e) {
  setTimeout(function () {
    throw e;
  });
}
function Il(e, t) {
  var n = t,
    r = 0;
  do {
    var i = n.nextSibling;
    if ((e.removeChild(n), i && i.nodeType === 8))
      if (((n = i.data), n === "/$")) {
        if (r === 0) {
          e.removeChild(i), si(t);
          return;
        }
        r--;
      } else (n !== "$" && n !== "$?" && n !== "$!") || r++;
    n = i;
  } while (n);
  si(t);
}
function Qt(e) {
  for (; e != null; e = e.nextSibling) {
    var t = e.nodeType;
    if (t === 1 || t === 3) break;
    if (t === 8) {
      if (((t = e.data), t === "$" || t === "$!" || t === "$?")) break;
      if (t === "/$") return null;
    }
  }
  return e;
}
function pc(e) {
  e = e.previousSibling;
  for (var t = 0; e; ) {
    if (e.nodeType === 8) {
      var n = e.data;
      if (n === "$" || n === "$!" || n === "$?") {
        if (t === 0) return e;
        t--;
      } else n === "/$" && t++;
    }
    e = e.previousSibling;
  }
  return null;
}
var vr = Math.random().toString(36).slice(2),
  mt = "__reactFiber$" + vr,
  di = "__reactProps$" + vr,
  _t = "__reactContainer$" + vr,
  Os = "__reactEvents$" + vr,
  bm = "__reactListeners$" + vr,
  Xm = "__reactHandles$" + vr;
function cn(e) {
  var t = e[mt];
  if (t) return t;
  for (var n = e.parentNode; n; ) {
    if ((t = n[_t] || n[mt])) {
      if (
        ((n = t.alternate),
        t.child !== null || (n !== null && n.child !== null))
      )
        for (e = pc(e); e !== null; ) {
          if ((n = e[mt])) return n;
          e = pc(e);
        }
      return t;
    }
    (e = n), (n = e.parentNode);
  }
  return null;
}
function _i(e) {
  return (
    (e = e[mt] || e[_t]),
    !e || (e.tag !== 5 && e.tag !== 6 && e.tag !== 13 && e.tag !== 3) ? null : e
  );
}
function Vn(e) {
  if (e.tag === 5 || e.tag === 6) return e.stateNode;
  throw Error(C(33));
}
function ol(e) {
  return e[di] || null;
}
var Rs = [],
  Bn = -1;
function en(e) {
  return { current: e };
}
function ee(e) {
  0 > Bn || ((e.current = Rs[Bn]), (Rs[Bn] = null), Bn--);
}
function J(e, t) {
  Bn++, (Rs[Bn] = e.current), (e.current = t);
}
var Jt = {},
  Ee = en(Jt),
  De = en(!1),
  vn = Jt;
function sr(e, t) {
  var n = e.type.contextTypes;
  if (!n) return Jt;
  var r = e.stateNode;
  if (r && r.__reactInternalMemoizedUnmaskedChildContext === t)
    return r.__reactInternalMemoizedMaskedChildContext;
  var i = {},
    o;
  for (o in n) i[o] = t[o];
  return (
    r &&
      ((e = e.stateNode),
      (e.__reactInternalMemoizedUnmaskedChildContext = t),
      (e.__reactInternalMemoizedMaskedChildContext = i)),
    i
  );
}
function ze(e) {
  return (e = e.childContextTypes), e != null;
}
function Co() {
  ee(De), ee(Ee);
}
function mc(e, t, n) {
  if (Ee.current !== Jt) throw Error(C(168));
  J(Ee, t), J(De, n);
}
function rh(e, t, n) {
  var r = e.stateNode;
  if (((t = t.childContextTypes), typeof r.getChildContext != "function"))
    return n;
  r = r.getChildContext();
  for (var i in r) if (!(i in t)) throw Error(C(108, A0(e) || "Unknown", i));
  return ie({}, n, r);
}
function Mo(e) {
  return (
    (e =
      ((e = e.stateNode) && e.__reactInternalMemoizedMergedChildContext) || Jt),
    (vn = Ee.current),
    J(Ee, e),
    J(De, De.current),
    !0
  );
}
function yc(e, t, n) {
  var r = e.stateNode;
  if (!r) throw Error(C(169));
  n
    ? ((e = rh(e, t, vn)),
      (r.__reactInternalMemoizedMergedChildContext = e),
      ee(De),
      ee(Ee),
      J(Ee, e))
    : ee(De),
    J(De, n);
}
var kt = null,
  ll = !1,
  Ul = !1;
function ih(e) {
  kt === null ? (kt = [e]) : kt.push(e);
}
function Km(e) {
  (ll = !0), ih(e);
}
function tn() {
  if (!Ul && kt !== null) {
    Ul = !0;
    var e = 0,
      t = G;
    try {
      var n = kt;
      for (G = 1; e < n.length; e++) {
        var r = n[e];
        do r = r(!0);
        while (r !== null);
      }
      (kt = null), (ll = !1);
    } catch (i) {
      throw (kt !== null && (kt = kt.slice(e + 1)), _d(_u, tn), i);
    } finally {
      (G = t), (Ul = !1);
    }
  }
  return null;
}
var Yn = [],
  Qn = 0,
  To = null,
  _o = 0,
  be = [],
  Xe = 0,
  wn = null,
  jt = 1,
  Et = "";
function on(e, t) {
  (Yn[Qn++] = _o), (Yn[Qn++] = To), (To = e), (_o = t);
}
function oh(e, t, n) {
  (be[Xe++] = jt), (be[Xe++] = Et), (be[Xe++] = wn), (wn = e);
  var r = jt;
  e = Et;
  var i = 32 - at(r) - 1;
  (r &= ~(1 << i)), (n += 1);
  var o = 32 - at(t) + i;
  if (30 < o) {
    var l = i - (i % 5);
    (o = (r & ((1 << l) - 1)).toString(32)),
      (r >>= l),
      (i -= l),
      (jt = (1 << (32 - at(t) + i)) | (n << i) | r),
      (Et = o + e);
  } else (jt = (1 << o) | (n << i) | r), (Et = e);
}
function Fu(e) {
  e.return !== null && (on(e, 1), oh(e, 1, 0));
}
function Iu(e) {
  for (; e === To; )
    (To = Yn[--Qn]), (Yn[Qn] = null), (_o = Yn[--Qn]), (Yn[Qn] = null);
  for (; e === wn; )
    (wn = be[--Xe]),
      (be[Xe] = null),
      (Et = be[--Xe]),
      (be[Xe] = null),
      (jt = be[--Xe]),
      (be[Xe] = null);
}
var He = null,
  Ue = null,
  te = !1,
  lt = null;
function lh(e, t) {
  var n = Ze(5, null, null, 0);
  (n.elementType = "DELETED"),
    (n.stateNode = t),
    (n.return = e),
    (t = e.deletions),
    t === null ? ((e.deletions = [n]), (e.flags |= 16)) : t.push(n);
}
function gc(e, t) {
  switch (e.tag) {
    case 5:
      var n = e.type;
      return (
        (t =
          t.nodeType !== 1 || n.toLowerCase() !== t.nodeName.toLowerCase()
            ? null
            : t),
        t !== null
          ? ((e.stateNode = t), (He = e), (Ue = Qt(t.firstChild)), !0)
          : !1
      );
    case 6:
      return (
        (t = e.pendingProps === "" || t.nodeType !== 3 ? null : t),
        t !== null ? ((e.stateNode = t), (He = e), (Ue = null), !0) : !1
      );
    case 13:
      return (
        (t = t.nodeType !== 8 ? null : t),
        t !== null
          ? ((n = wn !== null ? { id: jt, overflow: Et } : null),
            (e.memoizedState = {
              dehydrated: t,
              treeContext: n,
              retryLane: 1073741824,
            }),
            (n = Ze(18, null, null, 0)),
            (n.stateNode = t),
            (n.return = e),
            (e.child = n),
            (He = e),
            (Ue = null),
            !0)
          : !1
      );
    default:
      return !1;
  }
}
function Fs(e) {
  return (e.mode & 1) !== 0 && (e.flags & 128) === 0;
}
function Is(e) {
  if (te) {
    var t = Ue;
    if (t) {
      var n = t;
      if (!gc(e, t)) {
        if (Fs(e)) throw Error(C(418));
        t = Qt(n.nextSibling);
        var r = He;
        t && gc(e, t)
          ? lh(r, n)
          : ((e.flags = (e.flags & -4097) | 2), (te = !1), (He = e));
      }
    } else {
      if (Fs(e)) throw Error(C(418));
      (e.flags = (e.flags & -4097) | 2), (te = !1), (He = e);
    }
  }
}
function xc(e) {
  for (e = e.return; e !== null && e.tag !== 5 && e.tag !== 3 && e.tag !== 13; )
    e = e.return;
  He = e;
}
function Gi(e) {
  if (e !== He) return !1;
  if (!te) return xc(e), (te = !0), !1;
  var t;
  if (
    ((t = e.tag !== 3) &&
      !(t = e.tag !== 5) &&
      ((t = e.type),
      (t = t !== "head" && t !== "body" && !Ds(e.type, e.memoizedProps))),
    t && (t = Ue))
  ) {
    if (Fs(e)) throw (sh(), Error(C(418)));
    for (; t; ) lh(e, t), (t = Qt(t.nextSibling));
  }
  if ((xc(e), e.tag === 13)) {
    if (((e = e.memoizedState), (e = e !== null ? e.dehydrated : null), !e))
      throw Error(C(317));
    e: {
      for (e = e.nextSibling, t = 0; e; ) {
        if (e.nodeType === 8) {
          var n = e.data;
          if (n === "/$") {
            if (t === 0) {
              Ue = Qt(e.nextSibling);
              break e;
            }
            t--;
          } else (n !== "$" && n !== "$!" && n !== "$?") || t++;
        }
        e = e.nextSibling;
      }
      Ue = null;
    }
  } else Ue = He ? Qt(e.stateNode.nextSibling) : null;
  return !0;
}
function sh() {
  for (var e = Ue; e; ) e = Qt(e.nextSibling);
}
function ur() {
  (Ue = He = null), (te = !1);
}
function Uu(e) {
  lt === null ? (lt = [e]) : lt.push(e);
}
var Zm = Dt.ReactCurrentBatchConfig;
function _r(e, t, n) {
  if (
    ((e = n.ref), e !== null && typeof e != "function" && typeof e != "object")
  ) {
    if (n._owner) {
      if (((n = n._owner), n)) {
        if (n.tag !== 1) throw Error(C(309));
        var r = n.stateNode;
      }
      if (!r) throw Error(C(147, e));
      var i = r,
        o = "" + e;
      return t !== null &&
        t.ref !== null &&
        typeof t.ref == "function" &&
        t.ref._stringRef === o
        ? t.ref
        : ((t = function (l) {
            var s = i.refs;
            l === null ? delete s[o] : (s[o] = l);
          }),
          (t._stringRef = o),
          t);
    }
    if (typeof e != "string") throw Error(C(284));
    if (!n._owner) throw Error(C(290, e));
  }
  return e;
}
function bi(e, t) {
  throw (
    ((e = Object.prototype.toString.call(t)),
    Error(
      C(
        31,
        e === "[object Object]"
          ? "object with keys {" + Object.keys(t).join(", ") + "}"
          : e
      )
    ))
  );
}
function vc(e) {
  var t = e._init;
  return t(e._payload);
}
function uh(e) {
  function t(m, p) {
    if (e) {
      var y = m.deletions;
      y === null ? ((m.deletions = [p]), (m.flags |= 16)) : y.push(p);
    }
  }
  function n(m, p) {
    if (!e) return null;
    for (; p !== null; ) t(m, p), (p = p.sibling);
    return null;
  }
  function r(m, p) {
    for (m = new Map(); p !== null; )
      p.key !== null ? m.set(p.key, p) : m.set(p.index, p), (p = p.sibling);
    return m;
  }
  function i(m, p) {
    return (m = Kt(m, p)), (m.index = 0), (m.sibling = null), m;
  }
  function o(m, p, y) {
    return (
      (m.index = y),
      e
        ? ((y = m.alternate),
          y !== null
            ? ((y = y.index), y < p ? ((m.flags |= 2), p) : y)
            : ((m.flags |= 2), p))
        : ((m.flags |= 1048576), p)
    );
  }
  function l(m) {
    return e && m.alternate === null && (m.flags |= 2), m;
  }
  function s(m, p, y, w) {
    return p === null || p.tag !== 6
      ? ((p = Gl(y, m.mode, w)), (p.return = m), p)
      : ((p = i(p, y)), (p.return = m), p);
  }
  function u(m, p, y, w) {
    var S = y.type;
    return S === In
      ? f(m, p, y.props.children, w, y.key)
      : p !== null &&
        (p.elementType === S ||
          (typeof S == "object" &&
            S !== null &&
            S.$$typeof === Ot &&
            vc(S) === p.type))
      ? ((w = i(p, y.props)), (w.ref = _r(m, p, y)), (w.return = m), w)
      : ((w = mo(y.type, y.key, y.props, null, m.mode, w)),
        (w.ref = _r(m, p, y)),
        (w.return = m),
        w);
  }
  function a(m, p, y, w) {
    return p === null ||
      p.tag !== 4 ||
      p.stateNode.containerInfo !== y.containerInfo ||
      p.stateNode.implementation !== y.implementation
      ? ((p = bl(y, m.mode, w)), (p.return = m), p)
      : ((p = i(p, y.children || [])), (p.return = m), p);
  }
  function f(m, p, y, w, S) {
    return p === null || p.tag !== 7
      ? ((p = mn(y, m.mode, w, S)), (p.return = m), p)
      : ((p = i(p, y)), (p.return = m), p);
  }
  function d(m, p, y) {
    if ((typeof p == "string" && p !== "") || typeof p == "number")
      return (p = Gl("" + p, m.mode, y)), (p.return = m), p;
    if (typeof p == "object" && p !== null) {
      switch (p.$$typeof) {
        case Ri:
          return (
            (y = mo(p.type, p.key, p.props, null, m.mode, y)),
            (y.ref = _r(m, null, p)),
            (y.return = m),
            y
          );
        case Fn:
          return (p = bl(p, m.mode, y)), (p.return = m), p;
        case Ot:
          var w = p._init;
          return d(m, w(p._payload), y);
      }
      if (Ir(p) || Er(p))
        return (p = mn(p, m.mode, y, null)), (p.return = m), p;
      bi(m, p);
    }
    return null;
  }
  function h(m, p, y, w) {
    var S = p !== null ? p.key : null;
    if ((typeof y == "string" && y !== "") || typeof y == "number")
      return S !== null ? null : s(m, p, "" + y, w);
    if (typeof y == "object" && y !== null) {
      switch (y.$$typeof) {
        case Ri:
          return y.key === S ? u(m, p, y, w) : null;
        case Fn:
          return y.key === S ? a(m, p, y, w) : null;
        case Ot:
          return (S = y._init), h(m, p, S(y._payload), w);
      }
      if (Ir(y) || Er(y)) return S !== null ? null : f(m, p, y, w, null);
      bi(m, y);
    }
    return null;
  }
  function x(m, p, y, w, S) {
    if ((typeof w == "string" && w !== "") || typeof w == "number")
      return (m = m.get(y) || null), s(p, m, "" + w, S);
    if (typeof w == "object" && w !== null) {
      switch (w.$$typeof) {
        case Ri:
          return (m = m.get(w.key === null ? y : w.key) || null), u(p, m, w, S);
        case Fn:
          return (m = m.get(w.key === null ? y : w.key) || null), a(p, m, w, S);
        case Ot:
          var k = w._init;
          return x(m, p, y, k(w._payload), S);
      }
      if (Ir(w) || Er(w)) return (m = m.get(y) || null), f(p, m, w, S, null);
      bi(p, w);
    }
    return null;
  }
  function g(m, p, y, w) {
    for (
      var S = null, k = null, E = p, P = (p = 0), D = null;
      E !== null && P < y.length;
      P++
    ) {
      E.index > P ? ((D = E), (E = null)) : (D = E.sibling);
      var O = h(m, E, y[P], w);
      if (O === null) {
        E === null && (E = D);
        break;
      }
      e && E && O.alternate === null && t(m, E),
        (p = o(O, p, P)),
        k === null ? (S = O) : (k.sibling = O),
        (k = O),
        (E = D);
    }
    if (P === y.length) return n(m, E), te && on(m, P), S;
    if (E === null) {
      for (; P < y.length; P++)
        (E = d(m, y[P], w)),
          E !== null &&
            ((p = o(E, p, P)), k === null ? (S = E) : (k.sibling = E), (k = E));
      return te && on(m, P), S;
    }
    for (E = r(m, E); P < y.length; P++)
      (D = x(E, m, P, y[P], w)),
        D !== null &&
          (e && D.alternate !== null && E.delete(D.key === null ? P : D.key),
          (p = o(D, p, P)),
          k === null ? (S = D) : (k.sibling = D),
          (k = D));
    return (
      e &&
        E.forEach(function (N) {
          return t(m, N);
        }),
      te && on(m, P),
      S
    );
  }
  function v(m, p, y, w) {
    var S = Er(y);
    if (typeof S != "function") throw Error(C(150));
    if (((y = S.call(y)), y == null)) throw Error(C(151));
    for (
      var k = (S = null), E = p, P = (p = 0), D = null, O = y.next();
      E !== null && !O.done;
      P++, O = y.next()
    ) {
      E.index > P ? ((D = E), (E = null)) : (D = E.sibling);
      var N = h(m, E, O.value, w);
      if (N === null) {
        E === null && (E = D);
        break;
      }
      e && E && N.alternate === null && t(m, E),
        (p = o(N, p, P)),
        k === null ? (S = N) : (k.sibling = N),
        (k = N),
        (E = D);
    }
    if (O.done) return n(m, E), te && on(m, P), S;
    if (E === null) {
      for (; !O.done; P++, O = y.next())
        (O = d(m, O.value, w)),
          O !== null &&
            ((p = o(O, p, P)), k === null ? (S = O) : (k.sibling = O), (k = O));
      return te && on(m, P), S;
    }
    for (E = r(m, E); !O.done; P++, O = y.next())
      (O = x(E, m, P, O.value, w)),
        O !== null &&
          (e && O.alternate !== null && E.delete(O.key === null ? P : O.key),
          (p = o(O, p, P)),
          k === null ? (S = O) : (k.sibling = O),
          (k = O));
    return (
      e &&
        E.forEach(function (W) {
          return t(m, W);
        }),
      te && on(m, P),
      S
    );
  }
  function j(m, p, y, w) {
    if (
      (typeof y == "object" &&
        y !== null &&
        y.type === In &&
        y.key === null &&
        (y = y.props.children),
      typeof y == "object" && y !== null)
    ) {
      switch (y.$$typeof) {
        case Ri:
          e: {
            for (var S = y.key, k = p; k !== null; ) {
              if (k.key === S) {
                if (((S = y.type), S === In)) {
                  if (k.tag === 7) {
                    n(m, k.sibling),
                      (p = i(k, y.props.children)),
                      (p.return = m),
                      (m = p);
                    break e;
                  }
                } else if (
                  k.elementType === S ||
                  (typeof S == "object" &&
                    S !== null &&
                    S.$$typeof === Ot &&
                    vc(S) === k.type)
                ) {
                  n(m, k.sibling),
                    (p = i(k, y.props)),
                    (p.ref = _r(m, k, y)),
                    (p.return = m),
                    (m = p);
                  break e;
                }
                n(m, k);
                break;
              } else t(m, k);
              k = k.sibling;
            }
            y.type === In
              ? ((p = mn(y.props.children, m.mode, w, y.key)),
                (p.return = m),
                (m = p))
              : ((w = mo(y.type, y.key, y.props, null, m.mode, w)),
                (w.ref = _r(m, p, y)),
                (w.return = m),
                (m = w));
          }
          return l(m);
        case Fn:
          e: {
            for (k = y.key; p !== null; ) {
              if (p.key === k)
                if (
                  p.tag === 4 &&
                  p.stateNode.containerInfo === y.containerInfo &&
                  p.stateNode.implementation === y.implementation
                ) {
                  n(m, p.sibling),
                    (p = i(p, y.children || [])),
                    (p.return = m),
                    (m = p);
                  break e;
                } else {
                  n(m, p);
                  break;
                }
              else t(m, p);
              p = p.sibling;
            }
            (p = bl(y, m.mode, w)), (p.return = m), (m = p);
          }
          return l(m);
        case Ot:
          return (k = y._init), j(m, p, k(y._payload), w);
      }
      if (Ir(y)) return g(m, p, y, w);
      if (Er(y)) return v(m, p, y, w);
      bi(m, y);
    }
    return (typeof y == "string" && y !== "") || typeof y == "number"
      ? ((y = "" + y),
        p !== null && p.tag === 6
          ? (n(m, p.sibling), (p = i(p, y)), (p.return = m), (m = p))
          : (n(m, p), (p = Gl(y, m.mode, w)), (p.return = m), (m = p)),
        l(m))
      : n(m, p);
  }
  return j;
}
var ar = uh(!0),
  ah = uh(!1),
  Lo = en(null),
  No = null,
  Gn = null,
  Hu = null;
function Wu() {
  Hu = Gn = No = null;
}
function Vu(e) {
  var t = Lo.current;
  ee(Lo), (e._currentValue = t);
}
function Us(e, t, n) {
  for (; e !== null; ) {
    var r = e.alternate;
    if (
      ((e.childLanes & t) !== t
        ? ((e.childLanes |= t), r !== null && (r.childLanes |= t))
        : r !== null && (r.childLanes & t) !== t && (r.childLanes |= t),
      e === n)
    )
      break;
    e = e.return;
  }
}
function rr(e, t) {
  (No = e),
    (Hu = Gn = null),
    (e = e.dependencies),
    e !== null &&
      e.firstContext !== null &&
      (e.lanes & t && ($e = !0), (e.firstContext = null));
}
function et(e) {
  var t = e._currentValue;
  if (Hu !== e)
    if (((e = { context: e, memoizedValue: t, next: null }), Gn === null)) {
      if (No === null) throw Error(C(308));
      (Gn = e), (No.dependencies = { lanes: 0, firstContext: e });
    } else Gn = Gn.next = e;
  return t;
}
var fn = null;
function Bu(e) {
  fn === null ? (fn = [e]) : fn.push(e);
}
function ch(e, t, n, r) {
  var i = t.interleaved;
  return (
    i === null ? ((n.next = n), Bu(t)) : ((n.next = i.next), (i.next = n)),
    (t.interleaved = n),
    Lt(e, r)
  );
}
function Lt(e, t) {
  e.lanes |= t;
  var n = e.alternate;
  for (n !== null && (n.lanes |= t), n = e, e = e.return; e !== null; )
    (e.childLanes |= t),
      (n = e.alternate),
      n !== null && (n.childLanes |= t),
      (n = e),
      (e = e.return);
  return n.tag === 3 ? n.stateNode : null;
}
var Rt = !1;
function Yu(e) {
  e.updateQueue = {
    baseState: e.memoizedState,
    firstBaseUpdate: null,
    lastBaseUpdate: null,
    shared: { pending: null, interleaved: null, lanes: 0 },
    effects: null,
  };
}
function fh(e, t) {
  (e = e.updateQueue),
    t.updateQueue === e &&
      (t.updateQueue = {
        baseState: e.baseState,
        firstBaseUpdate: e.firstBaseUpdate,
        lastBaseUpdate: e.lastBaseUpdate,
        shared: e.shared,
        effects: e.effects,
      });
}
function Mt(e, t) {
  return {
    eventTime: e,
    lane: t,
    tag: 0,
    payload: null,
    callback: null,
    next: null,
  };
}
function Gt(e, t, n) {
  var r = e.updateQueue;
  if (r === null) return null;
  if (((r = r.shared), V & 2)) {
    var i = r.pending;
    return (
      i === null ? (t.next = t) : ((t.next = i.next), (i.next = t)),
      (r.pending = t),
      Lt(e, n)
    );
  }
  return (
    (i = r.interleaved),
    i === null ? ((t.next = t), Bu(r)) : ((t.next = i.next), (i.next = t)),
    (r.interleaved = t),
    Lt(e, n)
  );
}
function uo(e, t, n) {
  if (
    ((t = t.updateQueue), t !== null && ((t = t.shared), (n & 4194240) !== 0))
  ) {
    var r = t.lanes;
    (r &= e.pendingLanes), (n |= r), (t.lanes = n), Lu(e, n);
  }
}
function wc(e, t) {
  var n = e.updateQueue,
    r = e.alternate;
  if (r !== null && ((r = r.updateQueue), n === r)) {
    var i = null,
      o = null;
    if (((n = n.firstBaseUpdate), n !== null)) {
      do {
        var l = {
          eventTime: n.eventTime,
          lane: n.lane,
          tag: n.tag,
          payload: n.payload,
          callback: n.callback,
          next: null,
        };
        o === null ? (i = o = l) : (o = o.next = l), (n = n.next);
      } while (n !== null);
      o === null ? (i = o = t) : (o = o.next = t);
    } else i = o = t;
    (n = {
      baseState: r.baseState,
      firstBaseUpdate: i,
      lastBaseUpdate: o,
      shared: r.shared,
      effects: r.effects,
    }),
      (e.updateQueue = n);
    return;
  }
  (e = n.lastBaseUpdate),
    e === null ? (n.firstBaseUpdate = t) : (e.next = t),
    (n.lastBaseUpdate = t);
}
function $o(e, t, n, r) {
  var i = e.updateQueue;
  Rt = !1;
  var o = i.firstBaseUpdate,
    l = i.lastBaseUpdate,
    s = i.shared.pending;
  if (s !== null) {
    i.shared.pending = null;
    var u = s,
      a = u.next;
    (u.next = null), l === null ? (o = a) : (l.next = a), (l = u);
    var f = e.alternate;
    f !== null &&
      ((f = f.updateQueue),
      (s = f.lastBaseUpdate),
      s !== l &&
        (s === null ? (f.firstBaseUpdate = a) : (s.next = a),
        (f.lastBaseUpdate = u)));
  }
  if (o !== null) {
    var d = i.baseState;
    (l = 0), (f = a = u = null), (s = o);
    do {
      var h = s.lane,
        x = s.eventTime;
      if ((r & h) === h) {
        f !== null &&
          (f = f.next =
            {
              eventTime: x,
              lane: 0,
              tag: s.tag,
              payload: s.payload,
              callback: s.callback,
              next: null,
            });
        e: {
          var g = e,
            v = s;
          switch (((h = t), (x = n), v.tag)) {
            case 1:
              if (((g = v.payload), typeof g == "function")) {
                d = g.call(x, d, h);
                break e;
              }
              d = g;
              break e;
            case 3:
              g.flags = (g.flags & -65537) | 128;
            case 0:
              if (
                ((g = v.payload),
                (h = typeof g == "function" ? g.call(x, d, h) : g),
                h == null)
              )
                break e;
              d = ie({}, d, h);
              break e;
            case 2:
              Rt = !0;
          }
        }
        s.callback !== null &&
          s.lane !== 0 &&
          ((e.flags |= 64),
          (h = i.effects),
          h === null ? (i.effects = [s]) : h.push(s));
      } else
        (x = {
          eventTime: x,
          lane: h,
          tag: s.tag,
          payload: s.payload,
          callback: s.callback,
          next: null,
        }),
          f === null ? ((a = f = x), (u = d)) : (f = f.next = x),
          (l |= h);
      if (((s = s.next), s === null)) {
        if (((s = i.shared.pending), s === null)) break;
        (h = s),
          (s = h.next),
          (h.next = null),
          (i.lastBaseUpdate = h),
          (i.shared.pending = null);
      }
    } while (!0);
    if (
      (f === null && (u = d),
      (i.baseState = u),
      (i.firstBaseUpdate = a),
      (i.lastBaseUpdate = f),
      (t = i.shared.interleaved),
      t !== null)
    ) {
      i = t;
      do (l |= i.lane), (i = i.next);
      while (i !== t);
    } else o === null && (i.shared.lanes = 0);
    (kn |= l), (e.lanes = l), (e.memoizedState = d);
  }
}
function Sc(e, t, n) {
  if (((e = t.effects), (t.effects = null), e !== null))
    for (t = 0; t < e.length; t++) {
      var r = e[t],
        i = r.callback;
      if (i !== null) {
        if (((r.callback = null), (r = n), typeof i != "function"))
          throw Error(C(191, i));
        i.call(r);
      }
    }
}
var Li = {},
  gt = en(Li),
  hi = en(Li),
  pi = en(Li);
function dn(e) {
  if (e === Li) throw Error(C(174));
  return e;
}
function Qu(e, t) {
  switch ((J(pi, t), J(hi, e), J(gt, Li), (e = t.nodeType), e)) {
    case 9:
    case 11:
      t = (t = t.documentElement) ? t.namespaceURI : ws(null, "");
      break;
    default:
      (e = e === 8 ? t.parentNode : t),
        (t = e.namespaceURI || null),
        (e = e.tagName),
        (t = ws(t, e));
  }
  ee(gt), J(gt, t);
}
function cr() {
  ee(gt), ee(hi), ee(pi);
}
function dh(e) {
  dn(pi.current);
  var t = dn(gt.current),
    n = ws(t, e.type);
  t !== n && (J(hi, e), J(gt, n));
}
function Gu(e) {
  hi.current === e && (ee(gt), ee(hi));
}
var ne = en(0);
function Ao(e) {
  for (var t = e; t !== null; ) {
    if (t.tag === 13) {
      var n = t.memoizedState;
      if (
        n !== null &&
        ((n = n.dehydrated), n === null || n.data === "$?" || n.data === "$!")
      )
        return t;
    } else if (t.tag === 19 && t.memoizedProps.revealOrder !== void 0) {
      if (t.flags & 128) return t;
    } else if (t.child !== null) {
      (t.child.return = t), (t = t.child);
      continue;
    }
    if (t === e) break;
    for (; t.sibling === null; ) {
      if (t.return === null || t.return === e) return null;
      t = t.return;
    }
    (t.sibling.return = t.return), (t = t.sibling);
  }
  return null;
}
var Hl = [];
function bu() {
  for (var e = 0; e < Hl.length; e++)
    Hl[e]._workInProgressVersionPrimary = null;
  Hl.length = 0;
}
var ao = Dt.ReactCurrentDispatcher,
  Wl = Dt.ReactCurrentBatchConfig,
  Sn = 0,
  re = null,
  ce = null,
  pe = null,
  Do = !1,
  Zr = !1,
  mi = 0,
  Jm = 0;
function we() {
  throw Error(C(321));
}
function Xu(e, t) {
  if (t === null) return !1;
  for (var n = 0; n < t.length && n < e.length; n++)
    if (!ft(e[n], t[n])) return !1;
  return !0;
}
function Ku(e, t, n, r, i, o) {
  if (
    ((Sn = o),
    (re = t),
    (t.memoizedState = null),
    (t.updateQueue = null),
    (t.lanes = 0),
    (ao.current = e === null || e.memoizedState === null ? ny : ry),
    (e = n(r, i)),
    Zr)
  ) {
    o = 0;
    do {
      if (((Zr = !1), (mi = 0), 25 <= o)) throw Error(C(301));
      (o += 1),
        (pe = ce = null),
        (t.updateQueue = null),
        (ao.current = iy),
        (e = n(r, i));
    } while (Zr);
  }
  if (
    ((ao.current = zo),
    (t = ce !== null && ce.next !== null),
    (Sn = 0),
    (pe = ce = re = null),
    (Do = !1),
    t)
  )
    throw Error(C(300));
  return e;
}
function Zu() {
  var e = mi !== 0;
  return (mi = 0), e;
}
function ht() {
  var e = {
    memoizedState: null,
    baseState: null,
    baseQueue: null,
    queue: null,
    next: null,
  };
  return pe === null ? (re.memoizedState = pe = e) : (pe = pe.next = e), pe;
}
function tt() {
  if (ce === null) {
    var e = re.alternate;
    e = e !== null ? e.memoizedState : null;
  } else e = ce.next;
  var t = pe === null ? re.memoizedState : pe.next;
  if (t !== null) (pe = t), (ce = e);
  else {
    if (e === null) throw Error(C(310));
    (ce = e),
      (e = {
        memoizedState: ce.memoizedState,
        baseState: ce.baseState,
        baseQueue: ce.baseQueue,
        queue: ce.queue,
        next: null,
      }),
      pe === null ? (re.memoizedState = pe = e) : (pe = pe.next = e);
  }
  return pe;
}
function yi(e, t) {
  return typeof t == "function" ? t(e) : t;
}
function Vl(e) {
  var t = tt(),
    n = t.queue;
  if (n === null) throw Error(C(311));
  n.lastRenderedReducer = e;
  var r = ce,
    i = r.baseQueue,
    o = n.pending;
  if (o !== null) {
    if (i !== null) {
      var l = i.next;
      (i.next = o.next), (o.next = l);
    }
    (r.baseQueue = i = o), (n.pending = null);
  }
  if (i !== null) {
    (o = i.next), (r = r.baseState);
    var s = (l = null),
      u = null,
      a = o;
    do {
      var f = a.lane;
      if ((Sn & f) === f)
        u !== null &&
          (u = u.next =
            {
              lane: 0,
              action: a.action,
              hasEagerState: a.hasEagerState,
              eagerState: a.eagerState,
              next: null,
            }),
          (r = a.hasEagerState ? a.eagerState : e(r, a.action));
      else {
        var d = {
          lane: f,
          action: a.action,
          hasEagerState: a.hasEagerState,
          eagerState: a.eagerState,
          next: null,
        };
        u === null ? ((s = u = d), (l = r)) : (u = u.next = d),
          (re.lanes |= f),
          (kn |= f);
      }
      a = a.next;
    } while (a !== null && a !== o);
    u === null ? (l = r) : (u.next = s),
      ft(r, t.memoizedState) || ($e = !0),
      (t.memoizedState = r),
      (t.baseState = l),
      (t.baseQueue = u),
      (n.lastRenderedState = r);
  }
  if (((e = n.interleaved), e !== null)) {
    i = e;
    do (o = i.lane), (re.lanes |= o), (kn |= o), (i = i.next);
    while (i !== e);
  } else i === null && (n.lanes = 0);
  return [t.memoizedState, n.dispatch];
}
function Bl(e) {
  var t = tt(),
    n = t.queue;
  if (n === null) throw Error(C(311));
  n.lastRenderedReducer = e;
  var r = n.dispatch,
    i = n.pending,
    o = t.memoizedState;
  if (i !== null) {
    n.pending = null;
    var l = (i = i.next);
    do (o = e(o, l.action)), (l = l.next);
    while (l !== i);
    ft(o, t.memoizedState) || ($e = !0),
      (t.memoizedState = o),
      t.baseQueue === null && (t.baseState = o),
      (n.lastRenderedState = o);
  }
  return [o, r];
}
function hh() {}
function ph(e, t) {
  var n = re,
    r = tt(),
    i = t(),
    o = !ft(r.memoizedState, i);
  if (
    (o && ((r.memoizedState = i), ($e = !0)),
    (r = r.queue),
    Ju(gh.bind(null, n, r, e), [e]),
    r.getSnapshot !== t || o || (pe !== null && pe.memoizedState.tag & 1))
  ) {
    if (
      ((n.flags |= 2048),
      gi(9, yh.bind(null, n, r, i, t), void 0, null),
      me === null)
    )
      throw Error(C(349));
    Sn & 30 || mh(n, t, i);
  }
  return i;
}
function mh(e, t, n) {
  (e.flags |= 16384),
    (e = { getSnapshot: t, value: n }),
    (t = re.updateQueue),
    t === null
      ? ((t = { lastEffect: null, stores: null }),
        (re.updateQueue = t),
        (t.stores = [e]))
      : ((n = t.stores), n === null ? (t.stores = [e]) : n.push(e));
}
function yh(e, t, n, r) {
  (t.value = n), (t.getSnapshot = r), xh(t) && vh(e);
}
function gh(e, t, n) {
  return n(function () {
    xh(t) && vh(e);
  });
}
function xh(e) {
  var t = e.getSnapshot;
  e = e.value;
  try {
    var n = t();
    return !ft(e, n);
  } catch {
    return !0;
  }
}
function vh(e) {
  var t = Lt(e, 1);
  t !== null && ct(t, e, 1, -1);
}
function kc(e) {
  var t = ht();
  return (
    typeof e == "function" && (e = e()),
    (t.memoizedState = t.baseState = e),
    (e = {
      pending: null,
      interleaved: null,
      lanes: 0,
      dispatch: null,
      lastRenderedReducer: yi,
      lastRenderedState: e,
    }),
    (t.queue = e),
    (e = e.dispatch = ty.bind(null, re, e)),
    [t.memoizedState, e]
  );
}
function gi(e, t, n, r) {
  return (
    (e = { tag: e, create: t, destroy: n, deps: r, next: null }),
    (t = re.updateQueue),
    t === null
      ? ((t = { lastEffect: null, stores: null }),
        (re.updateQueue = t),
        (t.lastEffect = e.next = e))
      : ((n = t.lastEffect),
        n === null
          ? (t.lastEffect = e.next = e)
          : ((r = n.next), (n.next = e), (e.next = r), (t.lastEffect = e))),
    e
  );
}
function wh() {
  return tt().memoizedState;
}
function co(e, t, n, r) {
  var i = ht();
  (re.flags |= e),
    (i.memoizedState = gi(1 | t, n, void 0, r === void 0 ? null : r));
}
function sl(e, t, n, r) {
  var i = tt();
  r = r === void 0 ? null : r;
  var o = void 0;
  if (ce !== null) {
    var l = ce.memoizedState;
    if (((o = l.destroy), r !== null && Xu(r, l.deps))) {
      i.memoizedState = gi(t, n, o, r);
      return;
    }
  }
  (re.flags |= e), (i.memoizedState = gi(1 | t, n, o, r));
}
function jc(e, t) {
  return co(8390656, 8, e, t);
}
function Ju(e, t) {
  return sl(2048, 8, e, t);
}
function Sh(e, t) {
  return sl(4, 2, e, t);
}
function kh(e, t) {
  return sl(4, 4, e, t);
}
function jh(e, t) {
  if (typeof t == "function")
    return (
      (e = e()),
      t(e),
      function () {
        t(null);
      }
    );
  if (t != null)
    return (
      (e = e()),
      (t.current = e),
      function () {
        t.current = null;
      }
    );
}
function Eh(e, t, n) {
  return (
    (n = n != null ? n.concat([e]) : null), sl(4, 4, jh.bind(null, t, e), n)
  );
}
function qu() {}
function Ph(e, t) {
  var n = tt();
  t = t === void 0 ? null : t;
  var r = n.memoizedState;
  return r !== null && t !== null && Xu(t, r[1])
    ? r[0]
    : ((n.memoizedState = [e, t]), e);
}
function Ch(e, t) {
  var n = tt();
  t = t === void 0 ? null : t;
  var r = n.memoizedState;
  return r !== null && t !== null && Xu(t, r[1])
    ? r[0]
    : ((e = e()), (n.memoizedState = [e, t]), e);
}
function Mh(e, t, n) {
  return Sn & 21
    ? (ft(n, t) || ((n = $d()), (re.lanes |= n), (kn |= n), (e.baseState = !0)),
      t)
    : (e.baseState && ((e.baseState = !1), ($e = !0)), (e.memoizedState = n));
}
function qm(e, t) {
  var n = G;
  (G = n !== 0 && 4 > n ? n : 4), e(!0);
  var r = Wl.transition;
  Wl.transition = {};
  try {
    e(!1), t();
  } finally {
    (G = n), (Wl.transition = r);
  }
}
function Th() {
  return tt().memoizedState;
}
function ey(e, t, n) {
  var r = Xt(e);
  if (
    ((n = {
      lane: r,
      action: n,
      hasEagerState: !1,
      eagerState: null,
      next: null,
    }),
    _h(e))
  )
    Lh(t, n);
  else if (((n = ch(e, t, n, r)), n !== null)) {
    var i = Ce();
    ct(n, e, r, i), Nh(n, t, r);
  }
}
function ty(e, t, n) {
  var r = Xt(e),
    i = { lane: r, action: n, hasEagerState: !1, eagerState: null, next: null };
  if (_h(e)) Lh(t, i);
  else {
    var o = e.alternate;
    if (
      e.lanes === 0 &&
      (o === null || o.lanes === 0) &&
      ((o = t.lastRenderedReducer), o !== null)
    )
      try {
        var l = t.lastRenderedState,
          s = o(l, n);
        if (((i.hasEagerState = !0), (i.eagerState = s), ft(s, l))) {
          var u = t.interleaved;
          u === null
            ? ((i.next = i), Bu(t))
            : ((i.next = u.next), (u.next = i)),
            (t.interleaved = i);
          return;
        }
      } catch {
      } finally {
      }
    (n = ch(e, t, i, r)),
      n !== null && ((i = Ce()), ct(n, e, r, i), Nh(n, t, r));
  }
}
function _h(e) {
  var t = e.alternate;
  return e === re || (t !== null && t === re);
}
function Lh(e, t) {
  Zr = Do = !0;
  var n = e.pending;
  n === null ? (t.next = t) : ((t.next = n.next), (n.next = t)),
    (e.pending = t);
}
function Nh(e, t, n) {
  if (n & 4194240) {
    var r = t.lanes;
    (r &= e.pendingLanes), (n |= r), (t.lanes = n), Lu(e, n);
  }
}
var zo = {
    readContext: et,
    useCallback: we,
    useContext: we,
    useEffect: we,
    useImperativeHandle: we,
    useInsertionEffect: we,
    useLayoutEffect: we,
    useMemo: we,
    useReducer: we,
    useRef: we,
    useState: we,
    useDebugValue: we,
    useDeferredValue: we,
    useTransition: we,
    useMutableSource: we,
    useSyncExternalStore: we,
    useId: we,
    unstable_isNewReconciler: !1,
  },
  ny = {
    readContext: et,
    useCallback: function (e, t) {
      return (ht().memoizedState = [e, t === void 0 ? null : t]), e;
    },
    useContext: et,
    useEffect: jc,
    useImperativeHandle: function (e, t, n) {
      return (
        (n = n != null ? n.concat([e]) : null),
        co(4194308, 4, jh.bind(null, t, e), n)
      );
    },
    useLayoutEffect: function (e, t) {
      return co(4194308, 4, e, t);
    },
    useInsertionEffect: function (e, t) {
      return co(4, 2, e, t);
    },
    useMemo: function (e, t) {
      var n = ht();
      return (
        (t = t === void 0 ? null : t), (e = e()), (n.memoizedState = [e, t]), e
      );
    },
    useReducer: function (e, t, n) {
      var r = ht();
      return (
        (t = n !== void 0 ? n(t) : t),
        (r.memoizedState = r.baseState = t),
        (e = {
          pending: null,
          interleaved: null,
          lanes: 0,
          dispatch: null,
          lastRenderedReducer: e,
          lastRenderedState: t,
        }),
        (r.queue = e),
        (e = e.dispatch = ey.bind(null, re, e)),
        [r.memoizedState, e]
      );
    },
    useRef: function (e) {
      var t = ht();
      return (e = { current: e }), (t.memoizedState = e);
    },
    useState: kc,
    useDebugValue: qu,
    useDeferredValue: function (e) {
      return (ht().memoizedState = e);
    },
    useTransition: function () {
      var e = kc(!1),
        t = e[0];
      return (e = qm.bind(null, e[1])), (ht().memoizedState = e), [t, e];
    },
    useMutableSource: function () {},
    useSyncExternalStore: function (e, t, n) {
      var r = re,
        i = ht();
      if (te) {
        if (n === void 0) throw Error(C(407));
        n = n();
      } else {
        if (((n = t()), me === null)) throw Error(C(349));
        Sn & 30 || mh(r, t, n);
      }
      i.memoizedState = n;
      var o = { value: n, getSnapshot: t };
      return (
        (i.queue = o),
        jc(gh.bind(null, r, o, e), [e]),
        (r.flags |= 2048),
        gi(9, yh.bind(null, r, o, n, t), void 0, null),
        n
      );
    },
    useId: function () {
      var e = ht(),
        t = me.identifierPrefix;
      if (te) {
        var n = Et,
          r = jt;
        (n = (r & ~(1 << (32 - at(r) - 1))).toString(32) + n),
          (t = ":" + t + "R" + n),
          (n = mi++),
          0 < n && (t += "H" + n.toString(32)),
          (t += ":");
      } else (n = Jm++), (t = ":" + t + "r" + n.toString(32) + ":");
      return (e.memoizedState = t);
    },
    unstable_isNewReconciler: !1,
  },
  ry = {
    readContext: et,
    useCallback: Ph,
    useContext: et,
    useEffect: Ju,
    useImperativeHandle: Eh,
    useInsertionEffect: Sh,
    useLayoutEffect: kh,
    useMemo: Ch,
    useReducer: Vl,
    useRef: wh,
    useState: function () {
      return Vl(yi);
    },
    useDebugValue: qu,
    useDeferredValue: function (e) {
      var t = tt();
      return Mh(t, ce.memoizedState, e);
    },
    useTransition: function () {
      var e = Vl(yi)[0],
        t = tt().memoizedState;
      return [e, t];
    },
    useMutableSource: hh,
    useSyncExternalStore: ph,
    useId: Th,
    unstable_isNewReconciler: !1,
  },
  iy = {
    readContext: et,
    useCallback: Ph,
    useContext: et,
    useEffect: Ju,
    useImperativeHandle: Eh,
    useInsertionEffect: Sh,
    useLayoutEffect: kh,
    useMemo: Ch,
    useReducer: Bl,
    useRef: wh,
    useState: function () {
      return Bl(yi);
    },
    useDebugValue: qu,
    useDeferredValue: function (e) {
      var t = tt();
      return ce === null ? (t.memoizedState = e) : Mh(t, ce.memoizedState, e);
    },
    useTransition: function () {
      var e = Bl(yi)[0],
        t = tt().memoizedState;
      return [e, t];
    },
    useMutableSource: hh,
    useSyncExternalStore: ph,
    useId: Th,
    unstable_isNewReconciler: !1,
  };
function it(e, t) {
  if (e && e.defaultProps) {
    (t = ie({}, t)), (e = e.defaultProps);
    for (var n in e) t[n] === void 0 && (t[n] = e[n]);
    return t;
  }
  return t;
}
function Hs(e, t, n, r) {
  (t = e.memoizedState),
    (n = n(r, t)),
    (n = n == null ? t : ie({}, t, n)),
    (e.memoizedState = n),
    e.lanes === 0 && (e.updateQueue.baseState = n);
}
var ul = {
  isMounted: function (e) {
    return (e = e._reactInternals) ? _n(e) === e : !1;
  },
  enqueueSetState: function (e, t, n) {
    e = e._reactInternals;
    var r = Ce(),
      i = Xt(e),
      o = Mt(r, i);
    (o.payload = t),
      n != null && (o.callback = n),
      (t = Gt(e, o, i)),
      t !== null && (ct(t, e, i, r), uo(t, e, i));
  },
  enqueueReplaceState: function (e, t, n) {
    e = e._reactInternals;
    var r = Ce(),
      i = Xt(e),
      o = Mt(r, i);
    (o.tag = 1),
      (o.payload = t),
      n != null && (o.callback = n),
      (t = Gt(e, o, i)),
      t !== null && (ct(t, e, i, r), uo(t, e, i));
  },
  enqueueForceUpdate: function (e, t) {
    e = e._reactInternals;
    var n = Ce(),
      r = Xt(e),
      i = Mt(n, r);
    (i.tag = 2),
      t != null && (i.callback = t),
      (t = Gt(e, i, r)),
      t !== null && (ct(t, e, r, n), uo(t, e, r));
  },
};
function Ec(e, t, n, r, i, o, l) {
  return (
    (e = e.stateNode),
    typeof e.shouldComponentUpdate == "function"
      ? e.shouldComponentUpdate(r, o, l)
      : t.prototype && t.prototype.isPureReactComponent
      ? !ai(n, r) || !ai(i, o)
      : !0
  );
}
function $h(e, t, n) {
  var r = !1,
    i = Jt,
    o = t.contextType;
  return (
    typeof o == "object" && o !== null
      ? (o = et(o))
      : ((i = ze(t) ? vn : Ee.current),
        (r = t.contextTypes),
        (o = (r = r != null) ? sr(e, i) : Jt)),
    (t = new t(n, o)),
    (e.memoizedState = t.state !== null && t.state !== void 0 ? t.state : null),
    (t.updater = ul),
    (e.stateNode = t),
    (t._reactInternals = e),
    r &&
      ((e = e.stateNode),
      (e.__reactInternalMemoizedUnmaskedChildContext = i),
      (e.__reactInternalMemoizedMaskedChildContext = o)),
    t
  );
}
function Pc(e, t, n, r) {
  (e = t.state),
    typeof t.componentWillReceiveProps == "function" &&
      t.componentWillReceiveProps(n, r),
    typeof t.UNSAFE_componentWillReceiveProps == "function" &&
      t.UNSAFE_componentWillReceiveProps(n, r),
    t.state !== e && ul.enqueueReplaceState(t, t.state, null);
}
function Ws(e, t, n, r) {
  var i = e.stateNode;
  (i.props = n), (i.state = e.memoizedState), (i.refs = {}), Yu(e);
  var o = t.contextType;
  typeof o == "object" && o !== null
    ? (i.context = et(o))
    : ((o = ze(t) ? vn : Ee.current), (i.context = sr(e, o))),
    (i.state = e.memoizedState),
    (o = t.getDerivedStateFromProps),
    typeof o == "function" && (Hs(e, t, o, n), (i.state = e.memoizedState)),
    typeof t.getDerivedStateFromProps == "function" ||
      typeof i.getSnapshotBeforeUpdate == "function" ||
      (typeof i.UNSAFE_componentWillMount != "function" &&
        typeof i.componentWillMount != "function") ||
      ((t = i.state),
      typeof i.componentWillMount == "function" && i.componentWillMount(),
      typeof i.UNSAFE_componentWillMount == "function" &&
        i.UNSAFE_componentWillMount(),
      t !== i.state && ul.enqueueReplaceState(i, i.state, null),
      $o(e, n, i, r),
      (i.state = e.memoizedState)),
    typeof i.componentDidMount == "function" && (e.flags |= 4194308);
}
function fr(e, t) {
  try {
    var n = "",
      r = t;
    do (n += $0(r)), (r = r.return);
    while (r);
    var i = n;
  } catch (o) {
    i =
      `
Error generating stack: ` +
      o.message +
      `
` +
      o.stack;
  }
  return { value: e, source: t, stack: i, digest: null };
}
function Yl(e, t, n) {
  return { value: e, source: null, stack: n ?? null, digest: t ?? null };
}
function Vs(e, t) {
  try {
    console.error(t.value);
  } catch (n) {
    setTimeout(function () {
      throw n;
    });
  }
}
var oy = typeof WeakMap == "function" ? WeakMap : Map;
function Ah(e, t, n) {
  (n = Mt(-1, n)), (n.tag = 3), (n.payload = { element: null });
  var r = t.value;
  return (
    (n.callback = function () {
      Ro || ((Ro = !0), (qs = r)), Vs(e, t);
    }),
    n
  );
}
function Dh(e, t, n) {
  (n = Mt(-1, n)), (n.tag = 3);
  var r = e.type.getDerivedStateFromError;
  if (typeof r == "function") {
    var i = t.value;
    (n.payload = function () {
      return r(i);
    }),
      (n.callback = function () {
        Vs(e, t);
      });
  }
  var o = e.stateNode;
  return (
    o !== null &&
      typeof o.componentDidCatch == "function" &&
      (n.callback = function () {
        Vs(e, t),
          typeof r != "function" &&
            (bt === null ? (bt = new Set([this])) : bt.add(this));
        var l = t.stack;
        this.componentDidCatch(t.value, {
          componentStack: l !== null ? l : "",
        });
      }),
    n
  );
}
function Cc(e, t, n) {
  var r = e.pingCache;
  if (r === null) {
    r = e.pingCache = new oy();
    var i = new Set();
    r.set(t, i);
  } else (i = r.get(t)), i === void 0 && ((i = new Set()), r.set(t, i));
  i.has(n) || (i.add(n), (e = vy.bind(null, e, t, n)), t.then(e, e));
}
function Mc(e) {
  do {
    var t;
    if (
      ((t = e.tag === 13) &&
        ((t = e.memoizedState), (t = t !== null ? t.dehydrated !== null : !0)),
      t)
    )
      return e;
    e = e.return;
  } while (e !== null);
  return null;
}
function Tc(e, t, n, r, i) {
  return e.mode & 1
    ? ((e.flags |= 65536), (e.lanes = i), e)
    : (e === t
        ? (e.flags |= 65536)
        : ((e.flags |= 128),
          (n.flags |= 131072),
          (n.flags &= -52805),
          n.tag === 1 &&
            (n.alternate === null
              ? (n.tag = 17)
              : ((t = Mt(-1, 1)), (t.tag = 2), Gt(n, t, 1))),
          (n.lanes |= 1)),
      e);
}
var ly = Dt.ReactCurrentOwner,
  $e = !1;
function Pe(e, t, n, r) {
  t.child = e === null ? ah(t, null, n, r) : ar(t, e.child, n, r);
}
function _c(e, t, n, r, i) {
  n = n.render;
  var o = t.ref;
  return (
    rr(t, i),
    (r = Ku(e, t, n, r, o, i)),
    (n = Zu()),
    e !== null && !$e
      ? ((t.updateQueue = e.updateQueue),
        (t.flags &= -2053),
        (e.lanes &= ~i),
        Nt(e, t, i))
      : (te && n && Fu(t), (t.flags |= 1), Pe(e, t, r, i), t.child)
  );
}
function Lc(e, t, n, r, i) {
  if (e === null) {
    var o = n.type;
    return typeof o == "function" &&
      !sa(o) &&
      o.defaultProps === void 0 &&
      n.compare === null &&
      n.defaultProps === void 0
      ? ((t.tag = 15), (t.type = o), zh(e, t, o, r, i))
      : ((e = mo(n.type, null, r, t, t.mode, i)),
        (e.ref = t.ref),
        (e.return = t),
        (t.child = e));
  }
  if (((o = e.child), !(e.lanes & i))) {
    var l = o.memoizedProps;
    if (
      ((n = n.compare), (n = n !== null ? n : ai), n(l, r) && e.ref === t.ref)
    )
      return Nt(e, t, i);
  }
  return (
    (t.flags |= 1),
    (e = Kt(o, r)),
    (e.ref = t.ref),
    (e.return = t),
    (t.child = e)
  );
}
function zh(e, t, n, r, i) {
  if (e !== null) {
    var o = e.memoizedProps;
    if (ai(o, r) && e.ref === t.ref)
      if ((($e = !1), (t.pendingProps = r = o), (e.lanes & i) !== 0))
        e.flags & 131072 && ($e = !0);
      else return (t.lanes = e.lanes), Nt(e, t, i);
  }
  return Bs(e, t, n, r, i);
}
function Oh(e, t, n) {
  var r = t.pendingProps,
    i = r.children,
    o = e !== null ? e.memoizedState : null;
  if (r.mode === "hidden")
    if (!(t.mode & 1))
      (t.memoizedState = { baseLanes: 0, cachePool: null, transitions: null }),
        J(Xn, Ie),
        (Ie |= n);
    else {
      if (!(n & 1073741824))
        return (
          (e = o !== null ? o.baseLanes | n : n),
          (t.lanes = t.childLanes = 1073741824),
          (t.memoizedState = {
            baseLanes: e,
            cachePool: null,
            transitions: null,
          }),
          (t.updateQueue = null),
          J(Xn, Ie),
          (Ie |= e),
          null
        );
      (t.memoizedState = { baseLanes: 0, cachePool: null, transitions: null }),
        (r = o !== null ? o.baseLanes : n),
        J(Xn, Ie),
        (Ie |= r);
    }
  else
    o !== null ? ((r = o.baseLanes | n), (t.memoizedState = null)) : (r = n),
      J(Xn, Ie),
      (Ie |= r);
  return Pe(e, t, i, n), t.child;
}
function Rh(e, t) {
  var n = t.ref;
  ((e === null && n !== null) || (e !== null && e.ref !== n)) &&
    ((t.flags |= 512), (t.flags |= 2097152));
}
function Bs(e, t, n, r, i) {
  var o = ze(n) ? vn : Ee.current;
  return (
    (o = sr(t, o)),
    rr(t, i),
    (n = Ku(e, t, n, r, o, i)),
    (r = Zu()),
    e !== null && !$e
      ? ((t.updateQueue = e.updateQueue),
        (t.flags &= -2053),
        (e.lanes &= ~i),
        Nt(e, t, i))
      : (te && r && Fu(t), (t.flags |= 1), Pe(e, t, n, i), t.child)
  );
}
function Nc(e, t, n, r, i) {
  if (ze(n)) {
    var o = !0;
    Mo(t);
  } else o = !1;
  if ((rr(t, i), t.stateNode === null))
    fo(e, t), $h(t, n, r), Ws(t, n, r, i), (r = !0);
  else if (e === null) {
    var l = t.stateNode,
      s = t.memoizedProps;
    l.props = s;
    var u = l.context,
      a = n.contextType;
    typeof a == "object" && a !== null
      ? (a = et(a))
      : ((a = ze(n) ? vn : Ee.current), (a = sr(t, a)));
    var f = n.getDerivedStateFromProps,
      d =
        typeof f == "function" ||
        typeof l.getSnapshotBeforeUpdate == "function";
    d ||
      (typeof l.UNSAFE_componentWillReceiveProps != "function" &&
        typeof l.componentWillReceiveProps != "function") ||
      ((s !== r || u !== a) && Pc(t, l, r, a)),
      (Rt = !1);
    var h = t.memoizedState;
    (l.state = h),
      $o(t, r, l, i),
      (u = t.memoizedState),
      s !== r || h !== u || De.current || Rt
        ? (typeof f == "function" && (Hs(t, n, f, r), (u = t.memoizedState)),
          (s = Rt || Ec(t, n, s, r, h, u, a))
            ? (d ||
                (typeof l.UNSAFE_componentWillMount != "function" &&
                  typeof l.componentWillMount != "function") ||
                (typeof l.componentWillMount == "function" &&
                  l.componentWillMount(),
                typeof l.UNSAFE_componentWillMount == "function" &&
                  l.UNSAFE_componentWillMount()),
              typeof l.componentDidMount == "function" && (t.flags |= 4194308))
            : (typeof l.componentDidMount == "function" && (t.flags |= 4194308),
              (t.memoizedProps = r),
              (t.memoizedState = u)),
          (l.props = r),
          (l.state = u),
          (l.context = a),
          (r = s))
        : (typeof l.componentDidMount == "function" && (t.flags |= 4194308),
          (r = !1));
  } else {
    (l = t.stateNode),
      fh(e, t),
      (s = t.memoizedProps),
      (a = t.type === t.elementType ? s : it(t.type, s)),
      (l.props = a),
      (d = t.pendingProps),
      (h = l.context),
      (u = n.contextType),
      typeof u == "object" && u !== null
        ? (u = et(u))
        : ((u = ze(n) ? vn : Ee.current), (u = sr(t, u)));
    var x = n.getDerivedStateFromProps;
    (f =
      typeof x == "function" ||
      typeof l.getSnapshotBeforeUpdate == "function") ||
      (typeof l.UNSAFE_componentWillReceiveProps != "function" &&
        typeof l.componentWillReceiveProps != "function") ||
      ((s !== d || h !== u) && Pc(t, l, r, u)),
      (Rt = !1),
      (h = t.memoizedState),
      (l.state = h),
      $o(t, r, l, i);
    var g = t.memoizedState;
    s !== d || h !== g || De.current || Rt
      ? (typeof x == "function" && (Hs(t, n, x, r), (g = t.memoizedState)),
        (a = Rt || Ec(t, n, a, r, h, g, u) || !1)
          ? (f ||
              (typeof l.UNSAFE_componentWillUpdate != "function" &&
                typeof l.componentWillUpdate != "function") ||
              (typeof l.componentWillUpdate == "function" &&
                l.componentWillUpdate(r, g, u),
              typeof l.UNSAFE_componentWillUpdate == "function" &&
                l.UNSAFE_componentWillUpdate(r, g, u)),
            typeof l.componentDidUpdate == "function" && (t.flags |= 4),
            typeof l.getSnapshotBeforeUpdate == "function" && (t.flags |= 1024))
          : (typeof l.componentDidUpdate != "function" ||
              (s === e.memoizedProps && h === e.memoizedState) ||
              (t.flags |= 4),
            typeof l.getSnapshotBeforeUpdate != "function" ||
              (s === e.memoizedProps && h === e.memoizedState) ||
              (t.flags |= 1024),
            (t.memoizedProps = r),
            (t.memoizedState = g)),
        (l.props = r),
        (l.state = g),
        (l.context = u),
        (r = a))
      : (typeof l.componentDidUpdate != "function" ||
          (s === e.memoizedProps && h === e.memoizedState) ||
          (t.flags |= 4),
        typeof l.getSnapshotBeforeUpdate != "function" ||
          (s === e.memoizedProps && h === e.memoizedState) ||
          (t.flags |= 1024),
        (r = !1));
  }
  return Ys(e, t, n, r, o, i);
}
function Ys(e, t, n, r, i, o) {
  Rh(e, t);
  var l = (t.flags & 128) !== 0;
  if (!r && !l) return i && yc(t, n, !1), Nt(e, t, o);
  (r = t.stateNode), (ly.current = t);
  var s =
    l && typeof n.getDerivedStateFromError != "function" ? null : r.render();
  return (
    (t.flags |= 1),
    e !== null && l
      ? ((t.child = ar(t, e.child, null, o)), (t.child = ar(t, null, s, o)))
      : Pe(e, t, s, o),
    (t.memoizedState = r.state),
    i && yc(t, n, !0),
    t.child
  );
}
function Fh(e) {
  var t = e.stateNode;
  t.pendingContext
    ? mc(e, t.pendingContext, t.pendingContext !== t.context)
    : t.context && mc(e, t.context, !1),
    Qu(e, t.containerInfo);
}
function $c(e, t, n, r, i) {
  return ur(), Uu(i), (t.flags |= 256), Pe(e, t, n, r), t.child;
}
var Qs = { dehydrated: null, treeContext: null, retryLane: 0 };
function Gs(e) {
  return { baseLanes: e, cachePool: null, transitions: null };
}
function Ih(e, t, n) {
  var r = t.pendingProps,
    i = ne.current,
    o = !1,
    l = (t.flags & 128) !== 0,
    s;
  if (
    ((s = l) ||
      (s = e !== null && e.memoizedState === null ? !1 : (i & 2) !== 0),
    s
      ? ((o = !0), (t.flags &= -129))
      : (e === null || e.memoizedState !== null) && (i |= 1),
    J(ne, i & 1),
    e === null)
  )
    return (
      Is(t),
      (e = t.memoizedState),
      e !== null && ((e = e.dehydrated), e !== null)
        ? (t.mode & 1
            ? e.data === "$!"
              ? (t.lanes = 8)
              : (t.lanes = 1073741824)
            : (t.lanes = 1),
          null)
        : ((l = r.children),
          (e = r.fallback),
          o
            ? ((r = t.mode),
              (o = t.child),
              (l = { mode: "hidden", children: l }),
              !(r & 1) && o !== null
                ? ((o.childLanes = 0), (o.pendingProps = l))
                : (o = fl(l, r, 0, null)),
              (e = mn(e, r, n, null)),
              (o.return = t),
              (e.return = t),
              (o.sibling = e),
              (t.child = o),
              (t.child.memoizedState = Gs(n)),
              (t.memoizedState = Qs),
              e)
            : ea(t, l))
    );
  if (((i = e.memoizedState), i !== null && ((s = i.dehydrated), s !== null)))
    return sy(e, t, l, r, s, i, n);
  if (o) {
    (o = r.fallback), (l = t.mode), (i = e.child), (s = i.sibling);
    var u = { mode: "hidden", children: r.children };
    return (
      !(l & 1) && t.child !== i
        ? ((r = t.child),
          (r.childLanes = 0),
          (r.pendingProps = u),
          (t.deletions = null))
        : ((r = Kt(i, u)), (r.subtreeFlags = i.subtreeFlags & 14680064)),
      s !== null ? (o = Kt(s, o)) : ((o = mn(o, l, n, null)), (o.flags |= 2)),
      (o.return = t),
      (r.return = t),
      (r.sibling = o),
      (t.child = r),
      (r = o),
      (o = t.child),
      (l = e.child.memoizedState),
      (l =
        l === null
          ? Gs(n)
          : {
              baseLanes: l.baseLanes | n,
              cachePool: null,
              transitions: l.transitions,
            }),
      (o.memoizedState = l),
      (o.childLanes = e.childLanes & ~n),
      (t.memoizedState = Qs),
      r
    );
  }
  return (
    (o = e.child),
    (e = o.sibling),
    (r = Kt(o, { mode: "visible", children: r.children })),
    !(t.mode & 1) && (r.lanes = n),
    (r.return = t),
    (r.sibling = null),
    e !== null &&
      ((n = t.deletions),
      n === null ? ((t.deletions = [e]), (t.flags |= 16)) : n.push(e)),
    (t.child = r),
    (t.memoizedState = null),
    r
  );
}
function ea(e, t) {
  return (
    (t = fl({ mode: "visible", children: t }, e.mode, 0, null)),
    (t.return = e),
    (e.child = t)
  );
}
function Xi(e, t, n, r) {
  return (
    r !== null && Uu(r),
    ar(t, e.child, null, n),
    (e = ea(t, t.pendingProps.children)),
    (e.flags |= 2),
    (t.memoizedState = null),
    e
  );
}
function sy(e, t, n, r, i, o, l) {
  if (n)
    return t.flags & 256
      ? ((t.flags &= -257), (r = Yl(Error(C(422)))), Xi(e, t, l, r))
      : t.memoizedState !== null
      ? ((t.child = e.child), (t.flags |= 128), null)
      : ((o = r.fallback),
        (i = t.mode),
        (r = fl({ mode: "visible", children: r.children }, i, 0, null)),
        (o = mn(o, i, l, null)),
        (o.flags |= 2),
        (r.return = t),
        (o.return = t),
        (r.sibling = o),
        (t.child = r),
        t.mode & 1 && ar(t, e.child, null, l),
        (t.child.memoizedState = Gs(l)),
        (t.memoizedState = Qs),
        o);
  if (!(t.mode & 1)) return Xi(e, t, l, null);
  if (i.data === "$!") {
    if (((r = i.nextSibling && i.nextSibling.dataset), r)) var s = r.dgst;
    return (r = s), (o = Error(C(419))), (r = Yl(o, r, void 0)), Xi(e, t, l, r);
  }
  if (((s = (l & e.childLanes) !== 0), $e || s)) {
    if (((r = me), r !== null)) {
      switch (l & -l) {
        case 4:
          i = 2;
          break;
        case 16:
          i = 8;
          break;
        case 64:
        case 128:
        case 256:
        case 512:
        case 1024:
        case 2048:
        case 4096:
        case 8192:
        case 16384:
        case 32768:
        case 65536:
        case 131072:
        case 262144:
        case 524288:
        case 1048576:
        case 2097152:
        case 4194304:
        case 8388608:
        case 16777216:
        case 33554432:
        case 67108864:
          i = 32;
          break;
        case 536870912:
          i = 268435456;
          break;
        default:
          i = 0;
      }
      (i = i & (r.suspendedLanes | l) ? 0 : i),
        i !== 0 &&
          i !== o.retryLane &&
          ((o.retryLane = i), Lt(e, i), ct(r, e, i, -1));
    }
    return la(), (r = Yl(Error(C(421)))), Xi(e, t, l, r);
  }
  return i.data === "$?"
    ? ((t.flags |= 128),
      (t.child = e.child),
      (t = wy.bind(null, e)),
      (i._reactRetry = t),
      null)
    : ((e = o.treeContext),
      (Ue = Qt(i.nextSibling)),
      (He = t),
      (te = !0),
      (lt = null),
      e !== null &&
        ((be[Xe++] = jt),
        (be[Xe++] = Et),
        (be[Xe++] = wn),
        (jt = e.id),
        (Et = e.overflow),
        (wn = t)),
      (t = ea(t, r.children)),
      (t.flags |= 4096),
      t);
}
function Ac(e, t, n) {
  e.lanes |= t;
  var r = e.alternate;
  r !== null && (r.lanes |= t), Us(e.return, t, n);
}
function Ql(e, t, n, r, i) {
  var o = e.memoizedState;
  o === null
    ? (e.memoizedState = {
        isBackwards: t,
        rendering: null,
        renderingStartTime: 0,
        last: r,
        tail: n,
        tailMode: i,
      })
    : ((o.isBackwards = t),
      (o.rendering = null),
      (o.renderingStartTime = 0),
      (o.last = r),
      (o.tail = n),
      (o.tailMode = i));
}
function Uh(e, t, n) {
  var r = t.pendingProps,
    i = r.revealOrder,
    o = r.tail;
  if ((Pe(e, t, r.children, n), (r = ne.current), r & 2))
    (r = (r & 1) | 2), (t.flags |= 128);
  else {
    if (e !== null && e.flags & 128)
      e: for (e = t.child; e !== null; ) {
        if (e.tag === 13) e.memoizedState !== null && Ac(e, n, t);
        else if (e.tag === 19) Ac(e, n, t);
        else if (e.child !== null) {
          (e.child.return = e), (e = e.child);
          continue;
        }
        if (e === t) break e;
        for (; e.sibling === null; ) {
          if (e.return === null || e.return === t) break e;
          e = e.return;
        }
        (e.sibling.return = e.return), (e = e.sibling);
      }
    r &= 1;
  }
  if ((J(ne, r), !(t.mode & 1))) t.memoizedState = null;
  else
    switch (i) {
      case "forwards":
        for (n = t.child, i = null; n !== null; )
          (e = n.alternate),
            e !== null && Ao(e) === null && (i = n),
            (n = n.sibling);
        (n = i),
          n === null
            ? ((i = t.child), (t.child = null))
            : ((i = n.sibling), (n.sibling = null)),
          Ql(t, !1, i, n, o);
        break;
      case "backwards":
        for (n = null, i = t.child, t.child = null; i !== null; ) {
          if (((e = i.alternate), e !== null && Ao(e) === null)) {
            t.child = i;
            break;
          }
          (e = i.sibling), (i.sibling = n), (n = i), (i = e);
        }
        Ql(t, !0, n, null, o);
        break;
      case "together":
        Ql(t, !1, null, null, void 0);
        break;
      default:
        t.memoizedState = null;
    }
  return t.child;
}
function fo(e, t) {
  !(t.mode & 1) &&
    e !== null &&
    ((e.alternate = null), (t.alternate = null), (t.flags |= 2));
}
function Nt(e, t, n) {
  if (
    (e !== null && (t.dependencies = e.dependencies),
    (kn |= t.lanes),
    !(n & t.childLanes))
  )
    return null;
  if (e !== null && t.child !== e.child) throw Error(C(153));
  if (t.child !== null) {
    for (
      e = t.child, n = Kt(e, e.pendingProps), t.child = n, n.return = t;
      e.sibling !== null;

    )
      (e = e.sibling), (n = n.sibling = Kt(e, e.pendingProps)), (n.return = t);
    n.sibling = null;
  }
  return t.child;
}
function uy(e, t, n) {
  switch (t.tag) {
    case 3:
      Fh(t), ur();
      break;
    case 5:
      dh(t);
      break;
    case 1:
      ze(t.type) && Mo(t);
      break;
    case 4:
      Qu(t, t.stateNode.containerInfo);
      break;
    case 10:
      var r = t.type._context,
        i = t.memoizedProps.value;
      J(Lo, r._currentValue), (r._currentValue = i);
      break;
    case 13:
      if (((r = t.memoizedState), r !== null))
        return r.dehydrated !== null
          ? (J(ne, ne.current & 1), (t.flags |= 128), null)
          : n & t.child.childLanes
          ? Ih(e, t, n)
          : (J(ne, ne.current & 1),
            (e = Nt(e, t, n)),
            e !== null ? e.sibling : null);
      J(ne, ne.current & 1);
      break;
    case 19:
      if (((r = (n & t.childLanes) !== 0), e.flags & 128)) {
        if (r) return Uh(e, t, n);
        t.flags |= 128;
      }
      if (
        ((i = t.memoizedState),
        i !== null &&
          ((i.rendering = null), (i.tail = null), (i.lastEffect = null)),
        J(ne, ne.current),
        r)
      )
        break;
      return null;
    case 22:
    case 23:
      return (t.lanes = 0), Oh(e, t, n);
  }
  return Nt(e, t, n);
}
var Hh, bs, Wh, Vh;
Hh = function (e, t) {
  for (var n = t.child; n !== null; ) {
    if (n.tag === 5 || n.tag === 6) e.appendChild(n.stateNode);
    else if (n.tag !== 4 && n.child !== null) {
      (n.child.return = n), (n = n.child);
      continue;
    }
    if (n === t) break;
    for (; n.sibling === null; ) {
      if (n.return === null || n.return === t) return;
      n = n.return;
    }
    (n.sibling.return = n.return), (n = n.sibling);
  }
};
bs = function () {};
Wh = function (e, t, n, r) {
  var i = e.memoizedProps;
  if (i !== r) {
    (e = t.stateNode), dn(gt.current);
    var o = null;
    switch (n) {
      case "input":
        (i = ys(e, i)), (r = ys(e, r)), (o = []);
        break;
      case "select":
        (i = ie({}, i, { value: void 0 })),
          (r = ie({}, r, { value: void 0 })),
          (o = []);
        break;
      case "textarea":
        (i = vs(e, i)), (r = vs(e, r)), (o = []);
        break;
      default:
        typeof i.onClick != "function" &&
          typeof r.onClick == "function" &&
          (e.onclick = Po);
    }
    Ss(n, r);
    var l;
    n = null;
    for (a in i)
      if (!r.hasOwnProperty(a) && i.hasOwnProperty(a) && i[a] != null)
        if (a === "style") {
          var s = i[a];
          for (l in s) s.hasOwnProperty(l) && (n || (n = {}), (n[l] = ""));
        } else
          a !== "dangerouslySetInnerHTML" &&
            a !== "children" &&
            a !== "suppressContentEditableWarning" &&
            a !== "suppressHydrationWarning" &&
            a !== "autoFocus" &&
            (ni.hasOwnProperty(a)
              ? o || (o = [])
              : (o = o || []).push(a, null));
    for (a in r) {
      var u = r[a];
      if (
        ((s = i != null ? i[a] : void 0),
        r.hasOwnProperty(a) && u !== s && (u != null || s != null))
      )
        if (a === "style")
          if (s) {
            for (l in s)
              !s.hasOwnProperty(l) ||
                (u && u.hasOwnProperty(l)) ||
                (n || (n = {}), (n[l] = ""));
            for (l in u)
              u.hasOwnProperty(l) &&
                s[l] !== u[l] &&
                (n || (n = {}), (n[l] = u[l]));
          } else n || (o || (o = []), o.push(a, n)), (n = u);
        else
          a === "dangerouslySetInnerHTML"
            ? ((u = u ? u.__html : void 0),
              (s = s ? s.__html : void 0),
              u != null && s !== u && (o = o || []).push(a, u))
            : a === "children"
            ? (typeof u != "string" && typeof u != "number") ||
              (o = o || []).push(a, "" + u)
            : a !== "suppressContentEditableWarning" &&
              a !== "suppressHydrationWarning" &&
              (ni.hasOwnProperty(a)
                ? (u != null && a === "onScroll" && q("scroll", e),
                  o || s === u || (o = []))
                : (o = o || []).push(a, u));
    }
    n && (o = o || []).push("style", n);
    var a = o;
    (t.updateQueue = a) && (t.flags |= 4);
  }
};
Vh = function (e, t, n, r) {
  n !== r && (t.flags |= 4);
};
function Lr(e, t) {
  if (!te)
    switch (e.tailMode) {
      case "hidden":
        t = e.tail;
        for (var n = null; t !== null; )
          t.alternate !== null && (n = t), (t = t.sibling);
        n === null ? (e.tail = null) : (n.sibling = null);
        break;
      case "collapsed":
        n = e.tail;
        for (var r = null; n !== null; )
          n.alternate !== null && (r = n), (n = n.sibling);
        r === null
          ? t || e.tail === null
            ? (e.tail = null)
            : (e.tail.sibling = null)
          : (r.sibling = null);
    }
}
function Se(e) {
  var t = e.alternate !== null && e.alternate.child === e.child,
    n = 0,
    r = 0;
  if (t)
    for (var i = e.child; i !== null; )
      (n |= i.lanes | i.childLanes),
        (r |= i.subtreeFlags & 14680064),
        (r |= i.flags & 14680064),
        (i.return = e),
        (i = i.sibling);
  else
    for (i = e.child; i !== null; )
      (n |= i.lanes | i.childLanes),
        (r |= i.subtreeFlags),
        (r |= i.flags),
        (i.return = e),
        (i = i.sibling);
  return (e.subtreeFlags |= r), (e.childLanes = n), t;
}
function ay(e, t, n) {
  var r = t.pendingProps;
  switch ((Iu(t), t.tag)) {
    case 2:
    case 16:
    case 15:
    case 0:
    case 11:
    case 7:
    case 8:
    case 12:
    case 9:
    case 14:
      return Se(t), null;
    case 1:
      return ze(t.type) && Co(), Se(t), null;
    case 3:
      return (
        (r = t.stateNode),
        cr(),
        ee(De),
        ee(Ee),
        bu(),
        r.pendingContext &&
          ((r.context = r.pendingContext), (r.pendingContext = null)),
        (e === null || e.child === null) &&
          (Gi(t)
            ? (t.flags |= 4)
            : e === null ||
              (e.memoizedState.isDehydrated && !(t.flags & 256)) ||
              ((t.flags |= 1024), lt !== null && (nu(lt), (lt = null)))),
        bs(e, t),
        Se(t),
        null
      );
    case 5:
      Gu(t);
      var i = dn(pi.current);
      if (((n = t.type), e !== null && t.stateNode != null))
        Wh(e, t, n, r, i),
          e.ref !== t.ref && ((t.flags |= 512), (t.flags |= 2097152));
      else {
        if (!r) {
          if (t.stateNode === null) throw Error(C(166));
          return Se(t), null;
        }
        if (((e = dn(gt.current)), Gi(t))) {
          (r = t.stateNode), (n = t.type);
          var o = t.memoizedProps;
          switch (((r[mt] = t), (r[di] = o), (e = (t.mode & 1) !== 0), n)) {
            case "dialog":
              q("cancel", r), q("close", r);
              break;
            case "iframe":
            case "object":
            case "embed":
              q("load", r);
              break;
            case "video":
            case "audio":
              for (i = 0; i < Hr.length; i++) q(Hr[i], r);
              break;
            case "source":
              q("error", r);
              break;
            case "img":
            case "image":
            case "link":
              q("error", r), q("load", r);
              break;
            case "details":
              q("toggle", r);
              break;
            case "input":
              Wa(r, o), q("invalid", r);
              break;
            case "select":
              (r._wrapperState = { wasMultiple: !!o.multiple }),
                q("invalid", r);
              break;
            case "textarea":
              Ba(r, o), q("invalid", r);
          }
          Ss(n, o), (i = null);
          for (var l in o)
            if (o.hasOwnProperty(l)) {
              var s = o[l];
              l === "children"
                ? typeof s == "string"
                  ? r.textContent !== s &&
                    (o.suppressHydrationWarning !== !0 &&
                      Qi(r.textContent, s, e),
                    (i = ["children", s]))
                  : typeof s == "number" &&
                    r.textContent !== "" + s &&
                    (o.suppressHydrationWarning !== !0 &&
                      Qi(r.textContent, s, e),
                    (i = ["children", "" + s]))
                : ni.hasOwnProperty(l) &&
                  s != null &&
                  l === "onScroll" &&
                  q("scroll", r);
            }
          switch (n) {
            case "input":
              Fi(r), Va(r, o, !0);
              break;
            case "textarea":
              Fi(r), Ya(r);
              break;
            case "select":
            case "option":
              break;
            default:
              typeof o.onClick == "function" && (r.onclick = Po);
          }
          (r = i), (t.updateQueue = r), r !== null && (t.flags |= 4);
        } else {
          (l = i.nodeType === 9 ? i : i.ownerDocument),
            e === "http://www.w3.org/1999/xhtml" && (e = gd(n)),
            e === "http://www.w3.org/1999/xhtml"
              ? n === "script"
                ? ((e = l.createElement("div")),
                  (e.innerHTML = "<script></script>"),
                  (e = e.removeChild(e.firstChild)))
                : typeof r.is == "string"
                ? (e = l.createElement(n, { is: r.is }))
                : ((e = l.createElement(n)),
                  n === "select" &&
                    ((l = e),
                    r.multiple
                      ? (l.multiple = !0)
                      : r.size && (l.size = r.size)))
              : (e = l.createElementNS(e, n)),
            (e[mt] = t),
            (e[di] = r),
            Hh(e, t, !1, !1),
            (t.stateNode = e);
          e: {
            switch (((l = ks(n, r)), n)) {
              case "dialog":
                q("cancel", e), q("close", e), (i = r);
                break;
              case "iframe":
              case "object":
              case "embed":
                q("load", e), (i = r);
                break;
              case "video":
              case "audio":
                for (i = 0; i < Hr.length; i++) q(Hr[i], e);
                i = r;
                break;
              case "source":
                q("error", e), (i = r);
                break;
              case "img":
              case "image":
              case "link":
                q("error", e), q("load", e), (i = r);
                break;
              case "details":
                q("toggle", e), (i = r);
                break;
              case "input":
                Wa(e, r), (i = ys(e, r)), q("invalid", e);
                break;
              case "option":
                i = r;
                break;
              case "select":
                (e._wrapperState = { wasMultiple: !!r.multiple }),
                  (i = ie({}, r, { value: void 0 })),
                  q("invalid", e);
                break;
              case "textarea":
                Ba(e, r), (i = vs(e, r)), q("invalid", e);
                break;
              default:
                i = r;
            }
            Ss(n, i), (s = i);
            for (o in s)
              if (s.hasOwnProperty(o)) {
                var u = s[o];
                o === "style"
                  ? wd(e, u)
                  : o === "dangerouslySetInnerHTML"
                  ? ((u = u ? u.__html : void 0), u != null && xd(e, u))
                  : o === "children"
                  ? typeof u == "string"
                    ? (n !== "textarea" || u !== "") && ri(e, u)
                    : typeof u == "number" && ri(e, "" + u)
                  : o !== "suppressContentEditableWarning" &&
                    o !== "suppressHydrationWarning" &&
                    o !== "autoFocus" &&
                    (ni.hasOwnProperty(o)
                      ? u != null && o === "onScroll" && q("scroll", e)
                      : u != null && Eu(e, o, u, l));
              }
            switch (n) {
              case "input":
                Fi(e), Va(e, r, !1);
                break;
              case "textarea":
                Fi(e), Ya(e);
                break;
              case "option":
                r.value != null && e.setAttribute("value", "" + Zt(r.value));
                break;
              case "select":
                (e.multiple = !!r.multiple),
                  (o = r.value),
                  o != null
                    ? qn(e, !!r.multiple, o, !1)
                    : r.defaultValue != null &&
                      qn(e, !!r.multiple, r.defaultValue, !0);
                break;
              default:
                typeof i.onClick == "function" && (e.onclick = Po);
            }
            switch (n) {
              case "button":
              case "input":
              case "select":
              case "textarea":
                r = !!r.autoFocus;
                break e;
              case "img":
                r = !0;
                break e;
              default:
                r = !1;
            }
          }
          r && (t.flags |= 4);
        }
        t.ref !== null && ((t.flags |= 512), (t.flags |= 2097152));
      }
      return Se(t), null;
    case 6:
      if (e && t.stateNode != null) Vh(e, t, e.memoizedProps, r);
      else {
        if (typeof r != "string" && t.stateNode === null) throw Error(C(166));
        if (((n = dn(pi.current)), dn(gt.current), Gi(t))) {
          if (
            ((r = t.stateNode),
            (n = t.memoizedProps),
            (r[mt] = t),
            (o = r.nodeValue !== n) && ((e = He), e !== null))
          )
            switch (e.tag) {
              case 3:
                Qi(r.nodeValue, n, (e.mode & 1) !== 0);
                break;
              case 5:
                e.memoizedProps.suppressHydrationWarning !== !0 &&
                  Qi(r.nodeValue, n, (e.mode & 1) !== 0);
            }
          o && (t.flags |= 4);
        } else
          (r = (n.nodeType === 9 ? n : n.ownerDocument).createTextNode(r)),
            (r[mt] = t),
            (t.stateNode = r);
      }
      return Se(t), null;
    case 13:
      if (
        (ee(ne),
        (r = t.memoizedState),
        e === null ||
          (e.memoizedState !== null && e.memoizedState.dehydrated !== null))
      ) {
        if (te && Ue !== null && t.mode & 1 && !(t.flags & 128))
          sh(), ur(), (t.flags |= 98560), (o = !1);
        else if (((o = Gi(t)), r !== null && r.dehydrated !== null)) {
          if (e === null) {
            if (!o) throw Error(C(318));
            if (
              ((o = t.memoizedState),
              (o = o !== null ? o.dehydrated : null),
              !o)
            )
              throw Error(C(317));
            o[mt] = t;
          } else
            ur(), !(t.flags & 128) && (t.memoizedState = null), (t.flags |= 4);
          Se(t), (o = !1);
        } else lt !== null && (nu(lt), (lt = null)), (o = !0);
        if (!o) return t.flags & 65536 ? t : null;
      }
      return t.flags & 128
        ? ((t.lanes = n), t)
        : ((r = r !== null),
          r !== (e !== null && e.memoizedState !== null) &&
            r &&
            ((t.child.flags |= 8192),
            t.mode & 1 &&
              (e === null || ne.current & 1 ? fe === 0 && (fe = 3) : la())),
          t.updateQueue !== null && (t.flags |= 4),
          Se(t),
          null);
    case 4:
      return (
        cr(), bs(e, t), e === null && ci(t.stateNode.containerInfo), Se(t), null
      );
    case 10:
      return Vu(t.type._context), Se(t), null;
    case 17:
      return ze(t.type) && Co(), Se(t), null;
    case 19:
      if ((ee(ne), (o = t.memoizedState), o === null)) return Se(t), null;
      if (((r = (t.flags & 128) !== 0), (l = o.rendering), l === null))
        if (r) Lr(o, !1);
        else {
          if (fe !== 0 || (e !== null && e.flags & 128))
            for (e = t.child; e !== null; ) {
              if (((l = Ao(e)), l !== null)) {
                for (
                  t.flags |= 128,
                    Lr(o, !1),
                    r = l.updateQueue,
                    r !== null && ((t.updateQueue = r), (t.flags |= 4)),
                    t.subtreeFlags = 0,
                    r = n,
                    n = t.child;
                  n !== null;

                )
                  (o = n),
                    (e = r),
                    (o.flags &= 14680066),
                    (l = o.alternate),
                    l === null
                      ? ((o.childLanes = 0),
                        (o.lanes = e),
                        (o.child = null),
                        (o.subtreeFlags = 0),
                        (o.memoizedProps = null),
                        (o.memoizedState = null),
                        (o.updateQueue = null),
                        (o.dependencies = null),
                        (o.stateNode = null))
                      : ((o.childLanes = l.childLanes),
                        (o.lanes = l.lanes),
                        (o.child = l.child),
                        (o.subtreeFlags = 0),
                        (o.deletions = null),
                        (o.memoizedProps = l.memoizedProps),
                        (o.memoizedState = l.memoizedState),
                        (o.updateQueue = l.updateQueue),
                        (o.type = l.type),
                        (e = l.dependencies),
                        (o.dependencies =
                          e === null
                            ? null
                            : {
                                lanes: e.lanes,
                                firstContext: e.firstContext,
                              })),
                    (n = n.sibling);
                return J(ne, (ne.current & 1) | 2), t.child;
              }
              e = e.sibling;
            }
          o.tail !== null &&
            se() > dr &&
            ((t.flags |= 128), (r = !0), Lr(o, !1), (t.lanes = 4194304));
        }
      else {
        if (!r)
          if (((e = Ao(l)), e !== null)) {
            if (
              ((t.flags |= 128),
              (r = !0),
              (n = e.updateQueue),
              n !== null && ((t.updateQueue = n), (t.flags |= 4)),
              Lr(o, !0),
              o.tail === null && o.tailMode === "hidden" && !l.alternate && !te)
            )
              return Se(t), null;
          } else
            2 * se() - o.renderingStartTime > dr &&
              n !== 1073741824 &&
              ((t.flags |= 128), (r = !0), Lr(o, !1), (t.lanes = 4194304));
        o.isBackwards
          ? ((l.sibling = t.child), (t.child = l))
          : ((n = o.last),
            n !== null ? (n.sibling = l) : (t.child = l),
            (o.last = l));
      }
      return o.tail !== null
        ? ((t = o.tail),
          (o.rendering = t),
          (o.tail = t.sibling),
          (o.renderingStartTime = se()),
          (t.sibling = null),
          (n = ne.current),
          J(ne, r ? (n & 1) | 2 : n & 1),
          t)
        : (Se(t), null);
    case 22:
    case 23:
      return (
        oa(),
        (r = t.memoizedState !== null),
        e !== null && (e.memoizedState !== null) !== r && (t.flags |= 8192),
        r && t.mode & 1
          ? Ie & 1073741824 && (Se(t), t.subtreeFlags & 6 && (t.flags |= 8192))
          : Se(t),
        null
      );
    case 24:
      return null;
    case 25:
      return null;
  }
  throw Error(C(156, t.tag));
}
function cy(e, t) {
  switch ((Iu(t), t.tag)) {
    case 1:
      return (
        ze(t.type) && Co(),
        (e = t.flags),
        e & 65536 ? ((t.flags = (e & -65537) | 128), t) : null
      );
    case 3:
      return (
        cr(),
        ee(De),
        ee(Ee),
        bu(),
        (e = t.flags),
        e & 65536 && !(e & 128) ? ((t.flags = (e & -65537) | 128), t) : null
      );
    case 5:
      return Gu(t), null;
    case 13:
      if (
        (ee(ne), (e = t.memoizedState), e !== null && e.dehydrated !== null)
      ) {
        if (t.alternate === null) throw Error(C(340));
        ur();
      }
      return (
        (e = t.flags), e & 65536 ? ((t.flags = (e & -65537) | 128), t) : null
      );
    case 19:
      return ee(ne), null;
    case 4:
      return cr(), null;
    case 10:
      return Vu(t.type._context), null;
    case 22:
    case 23:
      return oa(), null;
    case 24:
      return null;
    default:
      return null;
  }
}
var Ki = !1,
  ke = !1,
  fy = typeof WeakSet == "function" ? WeakSet : Set,
  $ = null;
function bn(e, t) {
  var n = e.ref;
  if (n !== null)
    if (typeof n == "function")
      try {
        n(null);
      } catch (r) {
        oe(e, t, r);
      }
    else n.current = null;
}
function Xs(e, t, n) {
  try {
    n();
  } catch (r) {
    oe(e, t, r);
  }
}
var Dc = !1;
function dy(e, t) {
  if ((($s = ko), (e = bd()), Ru(e))) {
    if ("selectionStart" in e)
      var n = { start: e.selectionStart, end: e.selectionEnd };
    else
      e: {
        n = ((n = e.ownerDocument) && n.defaultView) || window;
        var r = n.getSelection && n.getSelection();
        if (r && r.rangeCount !== 0) {
          n = r.anchorNode;
          var i = r.anchorOffset,
            o = r.focusNode;
          r = r.focusOffset;
          try {
            n.nodeType, o.nodeType;
          } catch {
            n = null;
            break e;
          }
          var l = 0,
            s = -1,
            u = -1,
            a = 0,
            f = 0,
            d = e,
            h = null;
          t: for (;;) {
            for (
              var x;
              d !== n || (i !== 0 && d.nodeType !== 3) || (s = l + i),
                d !== o || (r !== 0 && d.nodeType !== 3) || (u = l + r),
                d.nodeType === 3 && (l += d.nodeValue.length),
                (x = d.firstChild) !== null;

            )
              (h = d), (d = x);
            for (;;) {
              if (d === e) break t;
              if (
                (h === n && ++a === i && (s = l),
                h === o && ++f === r && (u = l),
                (x = d.nextSibling) !== null)
              )
                break;
              (d = h), (h = d.parentNode);
            }
            d = x;
          }
          n = s === -1 || u === -1 ? null : { start: s, end: u };
        } else n = null;
      }
    n = n || { start: 0, end: 0 };
  } else n = null;
  for (As = { focusedElem: e, selectionRange: n }, ko = !1, $ = t; $ !== null; )
    if (((t = $), (e = t.child), (t.subtreeFlags & 1028) !== 0 && e !== null))
      (e.return = t), ($ = e);
    else
      for (; $ !== null; ) {
        t = $;
        try {
          var g = t.alternate;
          if (t.flags & 1024)
            switch (t.tag) {
              case 0:
              case 11:
              case 15:
                break;
              case 1:
                if (g !== null) {
                  var v = g.memoizedProps,
                    j = g.memoizedState,
                    m = t.stateNode,
                    p = m.getSnapshotBeforeUpdate(
                      t.elementType === t.type ? v : it(t.type, v),
                      j
                    );
                  m.__reactInternalSnapshotBeforeUpdate = p;
                }
                break;
              case 3:
                var y = t.stateNode.containerInfo;
                y.nodeType === 1
                  ? (y.textContent = "")
                  : y.nodeType === 9 &&
                    y.documentElement &&
                    y.removeChild(y.documentElement);
                break;
              case 5:
              case 6:
              case 4:
              case 17:
                break;
              default:
                throw Error(C(163));
            }
        } catch (w) {
          oe(t, t.return, w);
        }
        if (((e = t.sibling), e !== null)) {
          (e.return = t.return), ($ = e);
          break;
        }
        $ = t.return;
      }
  return (g = Dc), (Dc = !1), g;
}
function Jr(e, t, n) {
  var r = t.updateQueue;
  if (((r = r !== null ? r.lastEffect : null), r !== null)) {
    var i = (r = r.next);
    do {
      if ((i.tag & e) === e) {
        var o = i.destroy;
        (i.destroy = void 0), o !== void 0 && Xs(t, n, o);
      }
      i = i.next;
    } while (i !== r);
  }
}
function al(e, t) {
  if (
    ((t = t.updateQueue), (t = t !== null ? t.lastEffect : null), t !== null)
  ) {
    var n = (t = t.next);
    do {
      if ((n.tag & e) === e) {
        var r = n.create;
        n.destroy = r();
      }
      n = n.next;
    } while (n !== t);
  }
}
function Ks(e) {
  var t = e.ref;
  if (t !== null) {
    var n = e.stateNode;
    switch (e.tag) {
      case 5:
        e = n;
        break;
      default:
        e = n;
    }
    typeof t == "function" ? t(e) : (t.current = e);
  }
}
function Bh(e) {
  var t = e.alternate;
  t !== null && ((e.alternate = null), Bh(t)),
    (e.child = null),
    (e.deletions = null),
    (e.sibling = null),
    e.tag === 5 &&
      ((t = e.stateNode),
      t !== null &&
        (delete t[mt], delete t[di], delete t[Os], delete t[bm], delete t[Xm])),
    (e.stateNode = null),
    (e.return = null),
    (e.dependencies = null),
    (e.memoizedProps = null),
    (e.memoizedState = null),
    (e.pendingProps = null),
    (e.stateNode = null),
    (e.updateQueue = null);
}
function Yh(e) {
  return e.tag === 5 || e.tag === 3 || e.tag === 4;
}
function zc(e) {
  e: for (;;) {
    for (; e.sibling === null; ) {
      if (e.return === null || Yh(e.return)) return null;
      e = e.return;
    }
    for (
      e.sibling.return = e.return, e = e.sibling;
      e.tag !== 5 && e.tag !== 6 && e.tag !== 18;

    ) {
      if (e.flags & 2 || e.child === null || e.tag === 4) continue e;
      (e.child.return = e), (e = e.child);
    }
    if (!(e.flags & 2)) return e.stateNode;
  }
}
function Zs(e, t, n) {
  var r = e.tag;
  if (r === 5 || r === 6)
    (e = e.stateNode),
      t
        ? n.nodeType === 8
          ? n.parentNode.insertBefore(e, t)
          : n.insertBefore(e, t)
        : (n.nodeType === 8
            ? ((t = n.parentNode), t.insertBefore(e, n))
            : ((t = n), t.appendChild(e)),
          (n = n._reactRootContainer),
          n != null || t.onclick !== null || (t.onclick = Po));
  else if (r !== 4 && ((e = e.child), e !== null))
    for (Zs(e, t, n), e = e.sibling; e !== null; ) Zs(e, t, n), (e = e.sibling);
}
function Js(e, t, n) {
  var r = e.tag;
  if (r === 5 || r === 6)
    (e = e.stateNode), t ? n.insertBefore(e, t) : n.appendChild(e);
  else if (r !== 4 && ((e = e.child), e !== null))
    for (Js(e, t, n), e = e.sibling; e !== null; ) Js(e, t, n), (e = e.sibling);
}
var ge = null,
  ot = !1;
function zt(e, t, n) {
  for (n = n.child; n !== null; ) Qh(e, t, n), (n = n.sibling);
}
function Qh(e, t, n) {
  if (yt && typeof yt.onCommitFiberUnmount == "function")
    try {
      yt.onCommitFiberUnmount(tl, n);
    } catch {}
  switch (n.tag) {
    case 5:
      ke || bn(n, t);
    case 6:
      var r = ge,
        i = ot;
      (ge = null),
        zt(e, t, n),
        (ge = r),
        (ot = i),
        ge !== null &&
          (ot
            ? ((e = ge),
              (n = n.stateNode),
              e.nodeType === 8 ? e.parentNode.removeChild(n) : e.removeChild(n))
            : ge.removeChild(n.stateNode));
      break;
    case 18:
      ge !== null &&
        (ot
          ? ((e = ge),
            (n = n.stateNode),
            e.nodeType === 8
              ? Il(e.parentNode, n)
              : e.nodeType === 1 && Il(e, n),
            si(e))
          : Il(ge, n.stateNode));
      break;
    case 4:
      (r = ge),
        (i = ot),
        (ge = n.stateNode.containerInfo),
        (ot = !0),
        zt(e, t, n),
        (ge = r),
        (ot = i);
      break;
    case 0:
    case 11:
    case 14:
    case 15:
      if (
        !ke &&
        ((r = n.updateQueue), r !== null && ((r = r.lastEffect), r !== null))
      ) {
        i = r = r.next;
        do {
          var o = i,
            l = o.destroy;
          (o = o.tag),
            l !== void 0 && (o & 2 || o & 4) && Xs(n, t, l),
            (i = i.next);
        } while (i !== r);
      }
      zt(e, t, n);
      break;
    case 1:
      if (
        !ke &&
        (bn(n, t),
        (r = n.stateNode),
        typeof r.componentWillUnmount == "function")
      )
        try {
          (r.props = n.memoizedProps),
            (r.state = n.memoizedState),
            r.componentWillUnmount();
        } catch (s) {
          oe(n, t, s);
        }
      zt(e, t, n);
      break;
    case 21:
      zt(e, t, n);
      break;
    case 22:
      n.mode & 1
        ? ((ke = (r = ke) || n.memoizedState !== null), zt(e, t, n), (ke = r))
        : zt(e, t, n);
      break;
    default:
      zt(e, t, n);
  }
}
function Oc(e) {
  var t = e.updateQueue;
  if (t !== null) {
    e.updateQueue = null;
    var n = e.stateNode;
    n === null && (n = e.stateNode = new fy()),
      t.forEach(function (r) {
        var i = Sy.bind(null, e, r);
        n.has(r) || (n.add(r), r.then(i, i));
      });
  }
}
function rt(e, t) {
  var n = t.deletions;
  if (n !== null)
    for (var r = 0; r < n.length; r++) {
      var i = n[r];
      try {
        var o = e,
          l = t,
          s = l;
        e: for (; s !== null; ) {
          switch (s.tag) {
            case 5:
              (ge = s.stateNode), (ot = !1);
              break e;
            case 3:
              (ge = s.stateNode.containerInfo), (ot = !0);
              break e;
            case 4:
              (ge = s.stateNode.containerInfo), (ot = !0);
              break e;
          }
          s = s.return;
        }
        if (ge === null) throw Error(C(160));
        Qh(o, l, i), (ge = null), (ot = !1);
        var u = i.alternate;
        u !== null && (u.return = null), (i.return = null);
      } catch (a) {
        oe(i, t, a);
      }
    }
  if (t.subtreeFlags & 12854)
    for (t = t.child; t !== null; ) Gh(t, e), (t = t.sibling);
}
function Gh(e, t) {
  var n = e.alternate,
    r = e.flags;
  switch (e.tag) {
    case 0:
    case 11:
    case 14:
    case 15:
      if ((rt(t, e), dt(e), r & 4)) {
        try {
          Jr(3, e, e.return), al(3, e);
        } catch (v) {
          oe(e, e.return, v);
        }
        try {
          Jr(5, e, e.return);
        } catch (v) {
          oe(e, e.return, v);
        }
      }
      break;
    case 1:
      rt(t, e), dt(e), r & 512 && n !== null && bn(n, n.return);
      break;
    case 5:
      if (
        (rt(t, e),
        dt(e),
        r & 512 && n !== null && bn(n, n.return),
        e.flags & 32)
      ) {
        var i = e.stateNode;
        try {
          ri(i, "");
        } catch (v) {
          oe(e, e.return, v);
        }
      }
      if (r & 4 && ((i = e.stateNode), i != null)) {
        var o = e.memoizedProps,
          l = n !== null ? n.memoizedProps : o,
          s = e.type,
          u = e.updateQueue;
        if (((e.updateQueue = null), u !== null))
          try {
            s === "input" && o.type === "radio" && o.name != null && md(i, o),
              ks(s, l);
            var a = ks(s, o);
            for (l = 0; l < u.length; l += 2) {
              var f = u[l],
                d = u[l + 1];
              f === "style"
                ? wd(i, d)
                : f === "dangerouslySetInnerHTML"
                ? xd(i, d)
                : f === "children"
                ? ri(i, d)
                : Eu(i, f, d, a);
            }
            switch (s) {
              case "input":
                gs(i, o);
                break;
              case "textarea":
                yd(i, o);
                break;
              case "select":
                var h = i._wrapperState.wasMultiple;
                i._wrapperState.wasMultiple = !!o.multiple;
                var x = o.value;
                x != null
                  ? qn(i, !!o.multiple, x, !1)
                  : h !== !!o.multiple &&
                    (o.defaultValue != null
                      ? qn(i, !!o.multiple, o.defaultValue, !0)
                      : qn(i, !!o.multiple, o.multiple ? [] : "", !1));
            }
            i[di] = o;
          } catch (v) {
            oe(e, e.return, v);
          }
      }
      break;
    case 6:
      if ((rt(t, e), dt(e), r & 4)) {
        if (e.stateNode === null) throw Error(C(162));
        (i = e.stateNode), (o = e.memoizedProps);
        try {
          i.nodeValue = o;
        } catch (v) {
          oe(e, e.return, v);
        }
      }
      break;
    case 3:
      if (
        (rt(t, e), dt(e), r & 4 && n !== null && n.memoizedState.isDehydrated)
      )
        try {
          si(t.containerInfo);
        } catch (v) {
          oe(e, e.return, v);
        }
      break;
    case 4:
      rt(t, e), dt(e);
      break;
    case 13:
      rt(t, e),
        dt(e),
        (i = e.child),
        i.flags & 8192 &&
          ((o = i.memoizedState !== null),
          (i.stateNode.isHidden = o),
          !o ||
            (i.alternate !== null && i.alternate.memoizedState !== null) ||
            (ra = se())),
        r & 4 && Oc(e);
      break;
    case 22:
      if (
        ((f = n !== null && n.memoizedState !== null),
        e.mode & 1 ? ((ke = (a = ke) || f), rt(t, e), (ke = a)) : rt(t, e),
        dt(e),
        r & 8192)
      ) {
        if (
          ((a = e.memoizedState !== null),
          (e.stateNode.isHidden = a) && !f && e.mode & 1)
        )
          for ($ = e, f = e.child; f !== null; ) {
            for (d = $ = f; $ !== null; ) {
              switch (((h = $), (x = h.child), h.tag)) {
                case 0:
                case 11:
                case 14:
                case 15:
                  Jr(4, h, h.return);
                  break;
                case 1:
                  bn(h, h.return);
                  var g = h.stateNode;
                  if (typeof g.componentWillUnmount == "function") {
                    (r = h), (n = h.return);
                    try {
                      (t = r),
                        (g.props = t.memoizedProps),
                        (g.state = t.memoizedState),
                        g.componentWillUnmount();
                    } catch (v) {
                      oe(r, n, v);
                    }
                  }
                  break;
                case 5:
                  bn(h, h.return);
                  break;
                case 22:
                  if (h.memoizedState !== null) {
                    Fc(d);
                    continue;
                  }
              }
              x !== null ? ((x.return = h), ($ = x)) : Fc(d);
            }
            f = f.sibling;
          }
        e: for (f = null, d = e; ; ) {
          if (d.tag === 5) {
            if (f === null) {
              f = d;
              try {
                (i = d.stateNode),
                  a
                    ? ((o = i.style),
                      typeof o.setProperty == "function"
                        ? o.setProperty("display", "none", "important")
                        : (o.display = "none"))
                    : ((s = d.stateNode),
                      (u = d.memoizedProps.style),
                      (l =
                        u != null && u.hasOwnProperty("display")
                          ? u.display
                          : null),
                      (s.style.display = vd("display", l)));
              } catch (v) {
                oe(e, e.return, v);
              }
            }
          } else if (d.tag === 6) {
            if (f === null)
              try {
                d.stateNode.nodeValue = a ? "" : d.memoizedProps;
              } catch (v) {
                oe(e, e.return, v);
              }
          } else if (
            ((d.tag !== 22 && d.tag !== 23) ||
              d.memoizedState === null ||
              d === e) &&
            d.child !== null
          ) {
            (d.child.return = d), (d = d.child);
            continue;
          }
          if (d === e) break e;
          for (; d.sibling === null; ) {
            if (d.return === null || d.return === e) break e;
            f === d && (f = null), (d = d.return);
          }
          f === d && (f = null), (d.sibling.return = d.return), (d = d.sibling);
        }
      }
      break;
    case 19:
      rt(t, e), dt(e), r & 4 && Oc(e);
      break;
    case 21:
      break;
    default:
      rt(t, e), dt(e);
  }
}
function dt(e) {
  var t = e.flags;
  if (t & 2) {
    try {
      e: {
        for (var n = e.return; n !== null; ) {
          if (Yh(n)) {
            var r = n;
            break e;
          }
          n = n.return;
        }
        throw Error(C(160));
      }
      switch (r.tag) {
        case 5:
          var i = r.stateNode;
          r.flags & 32 && (ri(i, ""), (r.flags &= -33));
          var o = zc(e);
          Js(e, o, i);
          break;
        case 3:
        case 4:
          var l = r.stateNode.containerInfo,
            s = zc(e);
          Zs(e, s, l);
          break;
        default:
          throw Error(C(161));
      }
    } catch (u) {
      oe(e, e.return, u);
    }
    e.flags &= -3;
  }
  t & 4096 && (e.flags &= -4097);
}
function hy(e, t, n) {
  ($ = e), bh(e);
}
function bh(e, t, n) {
  for (var r = (e.mode & 1) !== 0; $ !== null; ) {
    var i = $,
      o = i.child;
    if (i.tag === 22 && r) {
      var l = i.memoizedState !== null || Ki;
      if (!l) {
        var s = i.alternate,
          u = (s !== null && s.memoizedState !== null) || ke;
        s = Ki;
        var a = ke;
        if (((Ki = l), (ke = u) && !a))
          for ($ = i; $ !== null; )
            (l = $),
              (u = l.child),
              l.tag === 22 && l.memoizedState !== null
                ? Ic(i)
                : u !== null
                ? ((u.return = l), ($ = u))
                : Ic(i);
        for (; o !== null; ) ($ = o), bh(o), (o = o.sibling);
        ($ = i), (Ki = s), (ke = a);
      }
      Rc(e);
    } else
      i.subtreeFlags & 8772 && o !== null ? ((o.return = i), ($ = o)) : Rc(e);
  }
}
function Rc(e) {
  for (; $ !== null; ) {
    var t = $;
    if (t.flags & 8772) {
      var n = t.alternate;
      try {
        if (t.flags & 8772)
          switch (t.tag) {
            case 0:
            case 11:
            case 15:
              ke || al(5, t);
              break;
            case 1:
              var r = t.stateNode;
              if (t.flags & 4 && !ke)
                if (n === null) r.componentDidMount();
                else {
                  var i =
                    t.elementType === t.type
                      ? n.memoizedProps
                      : it(t.type, n.memoizedProps);
                  r.componentDidUpdate(
                    i,
                    n.memoizedState,
                    r.__reactInternalSnapshotBeforeUpdate
                  );
                }
              var o = t.updateQueue;
              o !== null && Sc(t, o, r);
              break;
            case 3:
              var l = t.updateQueue;
              if (l !== null) {
                if (((n = null), t.child !== null))
                  switch (t.child.tag) {
                    case 5:
                      n = t.child.stateNode;
                      break;
                    case 1:
                      n = t.child.stateNode;
                  }
                Sc(t, l, n);
              }
              break;
            case 5:
              var s = t.stateNode;
              if (n === null && t.flags & 4) {
                n = s;
                var u = t.memoizedProps;
                switch (t.type) {
                  case "button":
                  case "input":
                  case "select":
                  case "textarea":
                    u.autoFocus && n.focus();
                    break;
                  case "img":
                    u.src && (n.src = u.src);
                }
              }
              break;
            case 6:
              break;
            case 4:
              break;
            case 12:
              break;
            case 13:
              if (t.memoizedState === null) {
                var a = t.alternate;
                if (a !== null) {
                  var f = a.memoizedState;
                  if (f !== null) {
                    var d = f.dehydrated;
                    d !== null && si(d);
                  }
                }
              }
              break;
            case 19:
            case 17:
            case 21:
            case 22:
            case 23:
            case 25:
              break;
            default:
              throw Error(C(163));
          }
        ke || (t.flags & 512 && Ks(t));
      } catch (h) {
        oe(t, t.return, h);
      }
    }
    if (t === e) {
      $ = null;
      break;
    }
    if (((n = t.sibling), n !== null)) {
      (n.return = t.return), ($ = n);
      break;
    }
    $ = t.return;
  }
}
function Fc(e) {
  for (; $ !== null; ) {
    var t = $;
    if (t === e) {
      $ = null;
      break;
    }
    var n = t.sibling;
    if (n !== null) {
      (n.return = t.return), ($ = n);
      break;
    }
    $ = t.return;
  }
}
function Ic(e) {
  for (; $ !== null; ) {
    var t = $;
    try {
      switch (t.tag) {
        case 0:
        case 11:
        case 15:
          var n = t.return;
          try {
            al(4, t);
          } catch (u) {
            oe(t, n, u);
          }
          break;
        case 1:
          var r = t.stateNode;
          if (typeof r.componentDidMount == "function") {
            var i = t.return;
            try {
              r.componentDidMount();
            } catch (u) {
              oe(t, i, u);
            }
          }
          var o = t.return;
          try {
            Ks(t);
          } catch (u) {
            oe(t, o, u);
          }
          break;
        case 5:
          var l = t.return;
          try {
            Ks(t);
          } catch (u) {
            oe(t, l, u);
          }
      }
    } catch (u) {
      oe(t, t.return, u);
    }
    if (t === e) {
      $ = null;
      break;
    }
    var s = t.sibling;
    if (s !== null) {
      (s.return = t.return), ($ = s);
      break;
    }
    $ = t.return;
  }
}
var py = Math.ceil,
  Oo = Dt.ReactCurrentDispatcher,
  ta = Dt.ReactCurrentOwner,
  qe = Dt.ReactCurrentBatchConfig,
  V = 0,
  me = null,
  ae = null,
  xe = 0,
  Ie = 0,
  Xn = en(0),
  fe = 0,
  xi = null,
  kn = 0,
  cl = 0,
  na = 0,
  qr = null,
  Ne = null,
  ra = 0,
  dr = 1 / 0,
  St = null,
  Ro = !1,
  qs = null,
  bt = null,
  Zi = !1,
  Wt = null,
  Fo = 0,
  ei = 0,
  eu = null,
  ho = -1,
  po = 0;
function Ce() {
  return V & 6 ? se() : ho !== -1 ? ho : (ho = se());
}
function Xt(e) {
  return e.mode & 1
    ? V & 2 && xe !== 0
      ? xe & -xe
      : Zm.transition !== null
      ? (po === 0 && (po = $d()), po)
      : ((e = G),
        e !== 0 || ((e = window.event), (e = e === void 0 ? 16 : Id(e.type))),
        e)
    : 1;
}
function ct(e, t, n, r) {
  if (50 < ei) throw ((ei = 0), (eu = null), Error(C(185)));
  Mi(e, n, r),
    (!(V & 2) || e !== me) &&
      (e === me && (!(V & 2) && (cl |= n), fe === 4 && Ut(e, xe)),
      Oe(e, r),
      n === 1 && V === 0 && !(t.mode & 1) && ((dr = se() + 500), ll && tn()));
}
function Oe(e, t) {
  var n = e.callbackNode;
  Z0(e, t);
  var r = So(e, e === me ? xe : 0);
  if (r === 0)
    n !== null && ba(n), (e.callbackNode = null), (e.callbackPriority = 0);
  else if (((t = r & -r), e.callbackPriority !== t)) {
    if ((n != null && ba(n), t === 1))
      e.tag === 0 ? Km(Uc.bind(null, e)) : ih(Uc.bind(null, e)),
        Qm(function () {
          !(V & 6) && tn();
        }),
        (n = null);
    else {
      switch (Ad(r)) {
        case 1:
          n = _u;
          break;
        case 4:
          n = Ld;
          break;
        case 16:
          n = wo;
          break;
        case 536870912:
          n = Nd;
          break;
        default:
          n = wo;
      }
      n = np(n, Xh.bind(null, e));
    }
    (e.callbackPriority = t), (e.callbackNode = n);
  }
}
function Xh(e, t) {
  if (((ho = -1), (po = 0), V & 6)) throw Error(C(327));
  var n = e.callbackNode;
  if (ir() && e.callbackNode !== n) return null;
  var r = So(e, e === me ? xe : 0);
  if (r === 0) return null;
  if (r & 30 || r & e.expiredLanes || t) t = Io(e, r);
  else {
    t = r;
    var i = V;
    V |= 2;
    var o = Zh();
    (me !== e || xe !== t) && ((St = null), (dr = se() + 500), pn(e, t));
    do
      try {
        gy();
        break;
      } catch (s) {
        Kh(e, s);
      }
    while (!0);
    Wu(),
      (Oo.current = o),
      (V = i),
      ae !== null ? (t = 0) : ((me = null), (xe = 0), (t = fe));
  }
  if (t !== 0) {
    if (
      (t === 2 && ((i = Ms(e)), i !== 0 && ((r = i), (t = tu(e, i)))), t === 1)
    )
      throw ((n = xi), pn(e, 0), Ut(e, r), Oe(e, se()), n);
    if (t === 6) Ut(e, r);
    else {
      if (
        ((i = e.current.alternate),
        !(r & 30) &&
          !my(i) &&
          ((t = Io(e, r)),
          t === 2 && ((o = Ms(e)), o !== 0 && ((r = o), (t = tu(e, o)))),
          t === 1))
      )
        throw ((n = xi), pn(e, 0), Ut(e, r), Oe(e, se()), n);
      switch (((e.finishedWork = i), (e.finishedLanes = r), t)) {
        case 0:
        case 1:
          throw Error(C(345));
        case 2:
          ln(e, Ne, St);
          break;
        case 3:
          if (
            (Ut(e, r), (r & 130023424) === r && ((t = ra + 500 - se()), 10 < t))
          ) {
            if (So(e, 0) !== 0) break;
            if (((i = e.suspendedLanes), (i & r) !== r)) {
              Ce(), (e.pingedLanes |= e.suspendedLanes & i);
              break;
            }
            e.timeoutHandle = zs(ln.bind(null, e, Ne, St), t);
            break;
          }
          ln(e, Ne, St);
          break;
        case 4:
          if ((Ut(e, r), (r & 4194240) === r)) break;
          for (t = e.eventTimes, i = -1; 0 < r; ) {
            var l = 31 - at(r);
            (o = 1 << l), (l = t[l]), l > i && (i = l), (r &= ~o);
          }
          if (
            ((r = i),
            (r = se() - r),
            (r =
              (120 > r
                ? 120
                : 480 > r
                ? 480
                : 1080 > r
                ? 1080
                : 1920 > r
                ? 1920
                : 3e3 > r
                ? 3e3
                : 4320 > r
                ? 4320
                : 1960 * py(r / 1960)) - r),
            10 < r)
          ) {
            e.timeoutHandle = zs(ln.bind(null, e, Ne, St), r);
            break;
          }
          ln(e, Ne, St);
          break;
        case 5:
          ln(e, Ne, St);
          break;
        default:
          throw Error(C(329));
      }
    }
  }
  return Oe(e, se()), e.callbackNode === n ? Xh.bind(null, e) : null;
}
function tu(e, t) {
  var n = qr;
  return (
    e.current.memoizedState.isDehydrated && (pn(e, t).flags |= 256),
    (e = Io(e, t)),
    e !== 2 && ((t = Ne), (Ne = n), t !== null && nu(t)),
    e
  );
}
function nu(e) {
  Ne === null ? (Ne = e) : Ne.push.apply(Ne, e);
}
function my(e) {
  for (var t = e; ; ) {
    if (t.flags & 16384) {
      var n = t.updateQueue;
      if (n !== null && ((n = n.stores), n !== null))
        for (var r = 0; r < n.length; r++) {
          var i = n[r],
            o = i.getSnapshot;
          i = i.value;
          try {
            if (!ft(o(), i)) return !1;
          } catch {
            return !1;
          }
        }
    }
    if (((n = t.child), t.subtreeFlags & 16384 && n !== null))
      (n.return = t), (t = n);
    else {
      if (t === e) break;
      for (; t.sibling === null; ) {
        if (t.return === null || t.return === e) return !0;
        t = t.return;
      }
      (t.sibling.return = t.return), (t = t.sibling);
    }
  }
  return !0;
}
function Ut(e, t) {
  for (
    t &= ~na,
      t &= ~cl,
      e.suspendedLanes |= t,
      e.pingedLanes &= ~t,
      e = e.expirationTimes;
    0 < t;

  ) {
    var n = 31 - at(t),
      r = 1 << n;
    (e[n] = -1), (t &= ~r);
  }
}
function Uc(e) {
  if (V & 6) throw Error(C(327));
  ir();
  var t = So(e, 0);
  if (!(t & 1)) return Oe(e, se()), null;
  var n = Io(e, t);
  if (e.tag !== 0 && n === 2) {
    var r = Ms(e);
    r !== 0 && ((t = r), (n = tu(e, r)));
  }
  if (n === 1) throw ((n = xi), pn(e, 0), Ut(e, t), Oe(e, se()), n);
  if (n === 6) throw Error(C(345));
  return (
    (e.finishedWork = e.current.alternate),
    (e.finishedLanes = t),
    ln(e, Ne, St),
    Oe(e, se()),
    null
  );
}
function ia(e, t) {
  var n = V;
  V |= 1;
  try {
    return e(t);
  } finally {
    (V = n), V === 0 && ((dr = se() + 500), ll && tn());
  }
}
function jn(e) {
  Wt !== null && Wt.tag === 0 && !(V & 6) && ir();
  var t = V;
  V |= 1;
  var n = qe.transition,
    r = G;
  try {
    if (((qe.transition = null), (G = 1), e)) return e();
  } finally {
    (G = r), (qe.transition = n), (V = t), !(V & 6) && tn();
  }
}
function oa() {
  (Ie = Xn.current), ee(Xn);
}
function pn(e, t) {
  (e.finishedWork = null), (e.finishedLanes = 0);
  var n = e.timeoutHandle;
  if ((n !== -1 && ((e.timeoutHandle = -1), Ym(n)), ae !== null))
    for (n = ae.return; n !== null; ) {
      var r = n;
      switch ((Iu(r), r.tag)) {
        case 1:
          (r = r.type.childContextTypes), r != null && Co();
          break;
        case 3:
          cr(), ee(De), ee(Ee), bu();
          break;
        case 5:
          Gu(r);
          break;
        case 4:
          cr();
          break;
        case 13:
          ee(ne);
          break;
        case 19:
          ee(ne);
          break;
        case 10:
          Vu(r.type._context);
          break;
        case 22:
        case 23:
          oa();
      }
      n = n.return;
    }
  if (
    ((me = e),
    (ae = e = Kt(e.current, null)),
    (xe = Ie = t),
    (fe = 0),
    (xi = null),
    (na = cl = kn = 0),
    (Ne = qr = null),
    fn !== null)
  ) {
    for (t = 0; t < fn.length; t++)
      if (((n = fn[t]), (r = n.interleaved), r !== null)) {
        n.interleaved = null;
        var i = r.next,
          o = n.pending;
        if (o !== null) {
          var l = o.next;
          (o.next = i), (r.next = l);
        }
        n.pending = r;
      }
    fn = null;
  }
  return e;
}
function Kh(e, t) {
  do {
    var n = ae;
    try {
      if ((Wu(), (ao.current = zo), Do)) {
        for (var r = re.memoizedState; r !== null; ) {
          var i = r.queue;
          i !== null && (i.pending = null), (r = r.next);
        }
        Do = !1;
      }
      if (
        ((Sn = 0),
        (pe = ce = re = null),
        (Zr = !1),
        (mi = 0),
        (ta.current = null),
        n === null || n.return === null)
      ) {
        (fe = 1), (xi = t), (ae = null);
        break;
      }
      e: {
        var o = e,
          l = n.return,
          s = n,
          u = t;
        if (
          ((t = xe),
          (s.flags |= 32768),
          u !== null && typeof u == "object" && typeof u.then == "function")
        ) {
          var a = u,
            f = s,
            d = f.tag;
          if (!(f.mode & 1) && (d === 0 || d === 11 || d === 15)) {
            var h = f.alternate;
            h
              ? ((f.updateQueue = h.updateQueue),
                (f.memoizedState = h.memoizedState),
                (f.lanes = h.lanes))
              : ((f.updateQueue = null), (f.memoizedState = null));
          }
          var x = Mc(l);
          if (x !== null) {
            (x.flags &= -257),
              Tc(x, l, s, o, t),
              x.mode & 1 && Cc(o, a, t),
              (t = x),
              (u = a);
            var g = t.updateQueue;
            if (g === null) {
              var v = new Set();
              v.add(u), (t.updateQueue = v);
            } else g.add(u);
            break e;
          } else {
            if (!(t & 1)) {
              Cc(o, a, t), la();
              break e;
            }
            u = Error(C(426));
          }
        } else if (te && s.mode & 1) {
          var j = Mc(l);
          if (j !== null) {
            !(j.flags & 65536) && (j.flags |= 256),
              Tc(j, l, s, o, t),
              Uu(fr(u, s));
            break e;
          }
        }
        (o = u = fr(u, s)),
          fe !== 4 && (fe = 2),
          qr === null ? (qr = [o]) : qr.push(o),
          (o = l);
        do {
          switch (o.tag) {
            case 3:
              (o.flags |= 65536), (t &= -t), (o.lanes |= t);
              var m = Ah(o, u, t);
              wc(o, m);
              break e;
            case 1:
              s = u;
              var p = o.type,
                y = o.stateNode;
              if (
                !(o.flags & 128) &&
                (typeof p.getDerivedStateFromError == "function" ||
                  (y !== null &&
                    typeof y.componentDidCatch == "function" &&
                    (bt === null || !bt.has(y))))
              ) {
                (o.flags |= 65536), (t &= -t), (o.lanes |= t);
                var w = Dh(o, s, t);
                wc(o, w);
                break e;
              }
          }
          o = o.return;
        } while (o !== null);
      }
      qh(n);
    } catch (S) {
      (t = S), ae === n && n !== null && (ae = n = n.return);
      continue;
    }
    break;
  } while (!0);
}
function Zh() {
  var e = Oo.current;
  return (Oo.current = zo), e === null ? zo : e;
}
function la() {
  (fe === 0 || fe === 3 || fe === 2) && (fe = 4),
    me === null || (!(kn & 268435455) && !(cl & 268435455)) || Ut(me, xe);
}
function Io(e, t) {
  var n = V;
  V |= 2;
  var r = Zh();
  (me !== e || xe !== t) && ((St = null), pn(e, t));
  do
    try {
      yy();
      break;
    } catch (i) {
      Kh(e, i);
    }
  while (!0);
  if ((Wu(), (V = n), (Oo.current = r), ae !== null)) throw Error(C(261));
  return (me = null), (xe = 0), fe;
}
function yy() {
  for (; ae !== null; ) Jh(ae);
}
function gy() {
  for (; ae !== null && !W0(); ) Jh(ae);
}
function Jh(e) {
  var t = tp(e.alternate, e, Ie);
  (e.memoizedProps = e.pendingProps),
    t === null ? qh(e) : (ae = t),
    (ta.current = null);
}
function qh(e) {
  var t = e;
  do {
    var n = t.alternate;
    if (((e = t.return), t.flags & 32768)) {
      if (((n = cy(n, t)), n !== null)) {
        (n.flags &= 32767), (ae = n);
        return;
      }
      if (e !== null)
        (e.flags |= 32768), (e.subtreeFlags = 0), (e.deletions = null);
      else {
        (fe = 6), (ae = null);
        return;
      }
    } else if (((n = ay(n, t, Ie)), n !== null)) {
      ae = n;
      return;
    }
    if (((t = t.sibling), t !== null)) {
      ae = t;
      return;
    }
    ae = t = e;
  } while (t !== null);
  fe === 0 && (fe = 5);
}
function ln(e, t, n) {
  var r = G,
    i = qe.transition;
  try {
    (qe.transition = null), (G = 1), xy(e, t, n, r);
  } finally {
    (qe.transition = i), (G = r);
  }
  return null;
}
function xy(e, t, n, r) {
  do ir();
  while (Wt !== null);
  if (V & 6) throw Error(C(327));
  n = e.finishedWork;
  var i = e.finishedLanes;
  if (n === null) return null;
  if (((e.finishedWork = null), (e.finishedLanes = 0), n === e.current))
    throw Error(C(177));
  (e.callbackNode = null), (e.callbackPriority = 0);
  var o = n.lanes | n.childLanes;
  if (
    (J0(e, o),
    e === me && ((ae = me = null), (xe = 0)),
    (!(n.subtreeFlags & 2064) && !(n.flags & 2064)) ||
      Zi ||
      ((Zi = !0),
      np(wo, function () {
        return ir(), null;
      })),
    (o = (n.flags & 15990) !== 0),
    n.subtreeFlags & 15990 || o)
  ) {
    (o = qe.transition), (qe.transition = null);
    var l = G;
    G = 1;
    var s = V;
    (V |= 4),
      (ta.current = null),
      dy(e, n),
      Gh(n, e),
      Fm(As),
      (ko = !!$s),
      (As = $s = null),
      (e.current = n),
      hy(n),
      V0(),
      (V = s),
      (G = l),
      (qe.transition = o);
  } else e.current = n;
  if (
    (Zi && ((Zi = !1), (Wt = e), (Fo = i)),
    (o = e.pendingLanes),
    o === 0 && (bt = null),
    Q0(n.stateNode),
    Oe(e, se()),
    t !== null)
  )
    for (r = e.onRecoverableError, n = 0; n < t.length; n++)
      (i = t[n]), r(i.value, { componentStack: i.stack, digest: i.digest });
  if (Ro) throw ((Ro = !1), (e = qs), (qs = null), e);
  return (
    Fo & 1 && e.tag !== 0 && ir(),
    (o = e.pendingLanes),
    o & 1 ? (e === eu ? ei++ : ((ei = 0), (eu = e))) : (ei = 0),
    tn(),
    null
  );
}
function ir() {
  if (Wt !== null) {
    var e = Ad(Fo),
      t = qe.transition,
      n = G;
    try {
      if (((qe.transition = null), (G = 16 > e ? 16 : e), Wt === null))
        var r = !1;
      else {
        if (((e = Wt), (Wt = null), (Fo = 0), V & 6)) throw Error(C(331));
        var i = V;
        for (V |= 4, $ = e.current; $ !== null; ) {
          var o = $,
            l = o.child;
          if ($.flags & 16) {
            var s = o.deletions;
            if (s !== null) {
              for (var u = 0; u < s.length; u++) {
                var a = s[u];
                for ($ = a; $ !== null; ) {
                  var f = $;
                  switch (f.tag) {
                    case 0:
                    case 11:
                    case 15:
                      Jr(8, f, o);
                  }
                  var d = f.child;
                  if (d !== null) (d.return = f), ($ = d);
                  else
                    for (; $ !== null; ) {
                      f = $;
                      var h = f.sibling,
                        x = f.return;
                      if ((Bh(f), f === a)) {
                        $ = null;
                        break;
                      }
                      if (h !== null) {
                        (h.return = x), ($ = h);
                        break;
                      }
                      $ = x;
                    }
                }
              }
              var g = o.alternate;
              if (g !== null) {
                var v = g.child;
                if (v !== null) {
                  g.child = null;
                  do {
                    var j = v.sibling;
                    (v.sibling = null), (v = j);
                  } while (v !== null);
                }
              }
              $ = o;
            }
          }
          if (o.subtreeFlags & 2064 && l !== null) (l.return = o), ($ = l);
          else
            e: for (; $ !== null; ) {
              if (((o = $), o.flags & 2048))
                switch (o.tag) {
                  case 0:
                  case 11:
                  case 15:
                    Jr(9, o, o.return);
                }
              var m = o.sibling;
              if (m !== null) {
                (m.return = o.return), ($ = m);
                break e;
              }
              $ = o.return;
            }
        }
        var p = e.current;
        for ($ = p; $ !== null; ) {
          l = $;
          var y = l.child;
          if (l.subtreeFlags & 2064 && y !== null) (y.return = l), ($ = y);
          else
            e: for (l = p; $ !== null; ) {
              if (((s = $), s.flags & 2048))
                try {
                  switch (s.tag) {
                    case 0:
                    case 11:
                    case 15:
                      al(9, s);
                  }
                } catch (S) {
                  oe(s, s.return, S);
                }
              if (s === l) {
                $ = null;
                break e;
              }
              var w = s.sibling;
              if (w !== null) {
                (w.return = s.return), ($ = w);
                break e;
              }
              $ = s.return;
            }
        }
        if (
          ((V = i), tn(), yt && typeof yt.onPostCommitFiberRoot == "function")
        )
          try {
            yt.onPostCommitFiberRoot(tl, e);
          } catch {}
        r = !0;
      }
      return r;
    } finally {
      (G = n), (qe.transition = t);
    }
  }
  return !1;
}
function Hc(e, t, n) {
  (t = fr(n, t)),
    (t = Ah(e, t, 1)),
    (e = Gt(e, t, 1)),
    (t = Ce()),
    e !== null && (Mi(e, 1, t), Oe(e, t));
}
function oe(e, t, n) {
  if (e.tag === 3) Hc(e, e, n);
  else
    for (; t !== null; ) {
      if (t.tag === 3) {
        Hc(t, e, n);
        break;
      } else if (t.tag === 1) {
        var r = t.stateNode;
        if (
          typeof t.type.getDerivedStateFromError == "function" ||
          (typeof r.componentDidCatch == "function" &&
            (bt === null || !bt.has(r)))
        ) {
          (e = fr(n, e)),
            (e = Dh(t, e, 1)),
            (t = Gt(t, e, 1)),
            (e = Ce()),
            t !== null && (Mi(t, 1, e), Oe(t, e));
          break;
        }
      }
      t = t.return;
    }
}
function vy(e, t, n) {
  var r = e.pingCache;
  r !== null && r.delete(t),
    (t = Ce()),
    (e.pingedLanes |= e.suspendedLanes & n),
    me === e &&
      (xe & n) === n &&
      (fe === 4 || (fe === 3 && (xe & 130023424) === xe && 500 > se() - ra)
        ? pn(e, 0)
        : (na |= n)),
    Oe(e, t);
}
function ep(e, t) {
  t === 0 &&
    (e.mode & 1
      ? ((t = Hi), (Hi <<= 1), !(Hi & 130023424) && (Hi = 4194304))
      : (t = 1));
  var n = Ce();
  (e = Lt(e, t)), e !== null && (Mi(e, t, n), Oe(e, n));
}
function wy(e) {
  var t = e.memoizedState,
    n = 0;
  t !== null && (n = t.retryLane), ep(e, n);
}
function Sy(e, t) {
  var n = 0;
  switch (e.tag) {
    case 13:
      var r = e.stateNode,
        i = e.memoizedState;
      i !== null && (n = i.retryLane);
      break;
    case 19:
      r = e.stateNode;
      break;
    default:
      throw Error(C(314));
  }
  r !== null && r.delete(t), ep(e, n);
}
var tp;
tp = function (e, t, n) {
  if (e !== null)
    if (e.memoizedProps !== t.pendingProps || De.current) $e = !0;
    else {
      if (!(e.lanes & n) && !(t.flags & 128)) return ($e = !1), uy(e, t, n);
      $e = !!(e.flags & 131072);
    }
  else ($e = !1), te && t.flags & 1048576 && oh(t, _o, t.index);
  switch (((t.lanes = 0), t.tag)) {
    case 2:
      var r = t.type;
      fo(e, t), (e = t.pendingProps);
      var i = sr(t, Ee.current);
      rr(t, n), (i = Ku(null, t, r, e, i, n));
      var o = Zu();
      return (
        (t.flags |= 1),
        typeof i == "object" &&
        i !== null &&
        typeof i.render == "function" &&
        i.$$typeof === void 0
          ? ((t.tag = 1),
            (t.memoizedState = null),
            (t.updateQueue = null),
            ze(r) ? ((o = !0), Mo(t)) : (o = !1),
            (t.memoizedState =
              i.state !== null && i.state !== void 0 ? i.state : null),
            Yu(t),
            (i.updater = ul),
            (t.stateNode = i),
            (i._reactInternals = t),
            Ws(t, r, e, n),
            (t = Ys(null, t, r, !0, o, n)))
          : ((t.tag = 0), te && o && Fu(t), Pe(null, t, i, n), (t = t.child)),
        t
      );
    case 16:
      r = t.elementType;
      e: {
        switch (
          (fo(e, t),
          (e = t.pendingProps),
          (i = r._init),
          (r = i(r._payload)),
          (t.type = r),
          (i = t.tag = jy(r)),
          (e = it(r, e)),
          i)
        ) {
          case 0:
            t = Bs(null, t, r, e, n);
            break e;
          case 1:
            t = Nc(null, t, r, e, n);
            break e;
          case 11:
            t = _c(null, t, r, e, n);
            break e;
          case 14:
            t = Lc(null, t, r, it(r.type, e), n);
            break e;
        }
        throw Error(C(306, r, ""));
      }
      return t;
    case 0:
      return (
        (r = t.type),
        (i = t.pendingProps),
        (i = t.elementType === r ? i : it(r, i)),
        Bs(e, t, r, i, n)
      );
    case 1:
      return (
        (r = t.type),
        (i = t.pendingProps),
        (i = t.elementType === r ? i : it(r, i)),
        Nc(e, t, r, i, n)
      );
    case 3:
      e: {
        if ((Fh(t), e === null)) throw Error(C(387));
        (r = t.pendingProps),
          (o = t.memoizedState),
          (i = o.element),
          fh(e, t),
          $o(t, r, null, n);
        var l = t.memoizedState;
        if (((r = l.element), o.isDehydrated))
          if (
            ((o = {
              element: r,
              isDehydrated: !1,
              cache: l.cache,
              pendingSuspenseBoundaries: l.pendingSuspenseBoundaries,
              transitions: l.transitions,
            }),
            (t.updateQueue.baseState = o),
            (t.memoizedState = o),
            t.flags & 256)
          ) {
            (i = fr(Error(C(423)), t)), (t = $c(e, t, r, n, i));
            break e;
          } else if (r !== i) {
            (i = fr(Error(C(424)), t)), (t = $c(e, t, r, n, i));
            break e;
          } else
            for (
              Ue = Qt(t.stateNode.containerInfo.firstChild),
                He = t,
                te = !0,
                lt = null,
                n = ah(t, null, r, n),
                t.child = n;
              n;

            )
              (n.flags = (n.flags & -3) | 4096), (n = n.sibling);
        else {
          if ((ur(), r === i)) {
            t = Nt(e, t, n);
            break e;
          }
          Pe(e, t, r, n);
        }
        t = t.child;
      }
      return t;
    case 5:
      return (
        dh(t),
        e === null && Is(t),
        (r = t.type),
        (i = t.pendingProps),
        (o = e !== null ? e.memoizedProps : null),
        (l = i.children),
        Ds(r, i) ? (l = null) : o !== null && Ds(r, o) && (t.flags |= 32),
        Rh(e, t),
        Pe(e, t, l, n),
        t.child
      );
    case 6:
      return e === null && Is(t), null;
    case 13:
      return Ih(e, t, n);
    case 4:
      return (
        Qu(t, t.stateNode.containerInfo),
        (r = t.pendingProps),
        e === null ? (t.child = ar(t, null, r, n)) : Pe(e, t, r, n),
        t.child
      );
    case 11:
      return (
        (r = t.type),
        (i = t.pendingProps),
        (i = t.elementType === r ? i : it(r, i)),
        _c(e, t, r, i, n)
      );
    case 7:
      return Pe(e, t, t.pendingProps, n), t.child;
    case 8:
      return Pe(e, t, t.pendingProps.children, n), t.child;
    case 12:
      return Pe(e, t, t.pendingProps.children, n), t.child;
    case 10:
      e: {
        if (
          ((r = t.type._context),
          (i = t.pendingProps),
          (o = t.memoizedProps),
          (l = i.value),
          J(Lo, r._currentValue),
          (r._currentValue = l),
          o !== null)
        )
          if (ft(o.value, l)) {
            if (o.children === i.children && !De.current) {
              t = Nt(e, t, n);
              break e;
            }
          } else
            for (o = t.child, o !== null && (o.return = t); o !== null; ) {
              var s = o.dependencies;
              if (s !== null) {
                l = o.child;
                for (var u = s.firstContext; u !== null; ) {
                  if (u.context === r) {
                    if (o.tag === 1) {
                      (u = Mt(-1, n & -n)), (u.tag = 2);
                      var a = o.updateQueue;
                      if (a !== null) {
                        a = a.shared;
                        var f = a.pending;
                        f === null
                          ? (u.next = u)
                          : ((u.next = f.next), (f.next = u)),
                          (a.pending = u);
                      }
                    }
                    (o.lanes |= n),
                      (u = o.alternate),
                      u !== null && (u.lanes |= n),
                      Us(o.return, n, t),
                      (s.lanes |= n);
                    break;
                  }
                  u = u.next;
                }
              } else if (o.tag === 10) l = o.type === t.type ? null : o.child;
              else if (o.tag === 18) {
                if (((l = o.return), l === null)) throw Error(C(341));
                (l.lanes |= n),
                  (s = l.alternate),
                  s !== null && (s.lanes |= n),
                  Us(l, n, t),
                  (l = o.sibling);
              } else l = o.child;
              if (l !== null) l.return = o;
              else
                for (l = o; l !== null; ) {
                  if (l === t) {
                    l = null;
                    break;
                  }
                  if (((o = l.sibling), o !== null)) {
                    (o.return = l.return), (l = o);
                    break;
                  }
                  l = l.return;
                }
              o = l;
            }
        Pe(e, t, i.children, n), (t = t.child);
      }
      return t;
    case 9:
      return (
        (i = t.type),
        (r = t.pendingProps.children),
        rr(t, n),
        (i = et(i)),
        (r = r(i)),
        (t.flags |= 1),
        Pe(e, t, r, n),
        t.child
      );
    case 14:
      return (
        (r = t.type),
        (i = it(r, t.pendingProps)),
        (i = it(r.type, i)),
        Lc(e, t, r, i, n)
      );
    case 15:
      return zh(e, t, t.type, t.pendingProps, n);
    case 17:
      return (
        (r = t.type),
        (i = t.pendingProps),
        (i = t.elementType === r ? i : it(r, i)),
        fo(e, t),
        (t.tag = 1),
        ze(r) ? ((e = !0), Mo(t)) : (e = !1),
        rr(t, n),
        $h(t, r, i),
        Ws(t, r, i, n),
        Ys(null, t, r, !0, e, n)
      );
    case 19:
      return Uh(e, t, n);
    case 22:
      return Oh(e, t, n);
  }
  throw Error(C(156, t.tag));
};
function np(e, t) {
  return _d(e, t);
}
function ky(e, t, n, r) {
  (this.tag = e),
    (this.key = n),
    (this.sibling =
      this.child =
      this.return =
      this.stateNode =
      this.type =
      this.elementType =
        null),
    (this.index = 0),
    (this.ref = null),
    (this.pendingProps = t),
    (this.dependencies =
      this.memoizedState =
      this.updateQueue =
      this.memoizedProps =
        null),
    (this.mode = r),
    (this.subtreeFlags = this.flags = 0),
    (this.deletions = null),
    (this.childLanes = this.lanes = 0),
    (this.alternate = null);
}
function Ze(e, t, n, r) {
  return new ky(e, t, n, r);
}
function sa(e) {
  return (e = e.prototype), !(!e || !e.isReactComponent);
}
function jy(e) {
  if (typeof e == "function") return sa(e) ? 1 : 0;
  if (e != null) {
    if (((e = e.$$typeof), e === Cu)) return 11;
    if (e === Mu) return 14;
  }
  return 2;
}
function Kt(e, t) {
  var n = e.alternate;
  return (
    n === null
      ? ((n = Ze(e.tag, t, e.key, e.mode)),
        (n.elementType = e.elementType),
        (n.type = e.type),
        (n.stateNode = e.stateNode),
        (n.alternate = e),
        (e.alternate = n))
      : ((n.pendingProps = t),
        (n.type = e.type),
        (n.flags = 0),
        (n.subtreeFlags = 0),
        (n.deletions = null)),
    (n.flags = e.flags & 14680064),
    (n.childLanes = e.childLanes),
    (n.lanes = e.lanes),
    (n.child = e.child),
    (n.memoizedProps = e.memoizedProps),
    (n.memoizedState = e.memoizedState),
    (n.updateQueue = e.updateQueue),
    (t = e.dependencies),
    (n.dependencies =
      t === null ? null : { lanes: t.lanes, firstContext: t.firstContext }),
    (n.sibling = e.sibling),
    (n.index = e.index),
    (n.ref = e.ref),
    n
  );
}
function mo(e, t, n, r, i, o) {
  var l = 2;
  if (((r = e), typeof e == "function")) sa(e) && (l = 1);
  else if (typeof e == "string") l = 5;
  else
    e: switch (e) {
      case In:
        return mn(n.children, i, o, t);
      case Pu:
        (l = 8), (i |= 8);
        break;
      case ds:
        return (
          (e = Ze(12, n, t, i | 2)), (e.elementType = ds), (e.lanes = o), e
        );
      case hs:
        return (e = Ze(13, n, t, i)), (e.elementType = hs), (e.lanes = o), e;
      case ps:
        return (e = Ze(19, n, t, i)), (e.elementType = ps), (e.lanes = o), e;
      case dd:
        return fl(n, i, o, t);
      default:
        if (typeof e == "object" && e !== null)
          switch (e.$$typeof) {
            case cd:
              l = 10;
              break e;
            case fd:
              l = 9;
              break e;
            case Cu:
              l = 11;
              break e;
            case Mu:
              l = 14;
              break e;
            case Ot:
              (l = 16), (r = null);
              break e;
          }
        throw Error(C(130, e == null ? e : typeof e, ""));
    }
  return (
    (t = Ze(l, n, t, i)), (t.elementType = e), (t.type = r), (t.lanes = o), t
  );
}
function mn(e, t, n, r) {
  return (e = Ze(7, e, r, t)), (e.lanes = n), e;
}
function fl(e, t, n, r) {
  return (
    (e = Ze(22, e, r, t)),
    (e.elementType = dd),
    (e.lanes = n),
    (e.stateNode = { isHidden: !1 }),
    e
  );
}
function Gl(e, t, n) {
  return (e = Ze(6, e, null, t)), (e.lanes = n), e;
}
function bl(e, t, n) {
  return (
    (t = Ze(4, e.children !== null ? e.children : [], e.key, t)),
    (t.lanes = n),
    (t.stateNode = {
      containerInfo: e.containerInfo,
      pendingChildren: null,
      implementation: e.implementation,
    }),
    t
  );
}
function Ey(e, t, n, r, i) {
  (this.tag = t),
    (this.containerInfo = e),
    (this.finishedWork =
      this.pingCache =
      this.current =
      this.pendingChildren =
        null),
    (this.timeoutHandle = -1),
    (this.callbackNode = this.pendingContext = this.context = null),
    (this.callbackPriority = 0),
    (this.eventTimes = Tl(0)),
    (this.expirationTimes = Tl(-1)),
    (this.entangledLanes =
      this.finishedLanes =
      this.mutableReadLanes =
      this.expiredLanes =
      this.pingedLanes =
      this.suspendedLanes =
      this.pendingLanes =
        0),
    (this.entanglements = Tl(0)),
    (this.identifierPrefix = r),
    (this.onRecoverableError = i),
    (this.mutableSourceEagerHydrationData = null);
}
function ua(e, t, n, r, i, o, l, s, u) {
  return (
    (e = new Ey(e, t, n, s, u)),
    t === 1 ? ((t = 1), o === !0 && (t |= 8)) : (t = 0),
    (o = Ze(3, null, null, t)),
    (e.current = o),
    (o.stateNode = e),
    (o.memoizedState = {
      element: r,
      isDehydrated: n,
      cache: null,
      transitions: null,
      pendingSuspenseBoundaries: null,
    }),
    Yu(o),
    e
  );
}
function Py(e, t, n) {
  var r = 3 < arguments.length && arguments[3] !== void 0 ? arguments[3] : null;
  return {
    $$typeof: Fn,
    key: r == null ? null : "" + r,
    children: e,
    containerInfo: t,
    implementation: n,
  };
}
function rp(e) {
  if (!e) return Jt;
  e = e._reactInternals;
  e: {
    if (_n(e) !== e || e.tag !== 1) throw Error(C(170));
    var t = e;
    do {
      switch (t.tag) {
        case 3:
          t = t.stateNode.context;
          break e;
        case 1:
          if (ze(t.type)) {
            t = t.stateNode.__reactInternalMemoizedMergedChildContext;
            break e;
          }
      }
      t = t.return;
    } while (t !== null);
    throw Error(C(171));
  }
  if (e.tag === 1) {
    var n = e.type;
    if (ze(n)) return rh(e, n, t);
  }
  return t;
}
function ip(e, t, n, r, i, o, l, s, u) {
  return (
    (e = ua(n, r, !0, e, i, o, l, s, u)),
    (e.context = rp(null)),
    (n = e.current),
    (r = Ce()),
    (i = Xt(n)),
    (o = Mt(r, i)),
    (o.callback = t ?? null),
    Gt(n, o, i),
    (e.current.lanes = i),
    Mi(e, i, r),
    Oe(e, r),
    e
  );
}
function dl(e, t, n, r) {
  var i = t.current,
    o = Ce(),
    l = Xt(i);
  return (
    (n = rp(n)),
    t.context === null ? (t.context = n) : (t.pendingContext = n),
    (t = Mt(o, l)),
    (t.payload = { element: e }),
    (r = r === void 0 ? null : r),
    r !== null && (t.callback = r),
    (e = Gt(i, t, l)),
    e !== null && (ct(e, i, l, o), uo(e, i, l)),
    l
  );
}
function Uo(e) {
  if (((e = e.current), !e.child)) return null;
  switch (e.child.tag) {
    case 5:
      return e.child.stateNode;
    default:
      return e.child.stateNode;
  }
}
function Wc(e, t) {
  if (((e = e.memoizedState), e !== null && e.dehydrated !== null)) {
    var n = e.retryLane;
    e.retryLane = n !== 0 && n < t ? n : t;
  }
}
function aa(e, t) {
  Wc(e, t), (e = e.alternate) && Wc(e, t);
}
function Cy() {
  return null;
}
var op =
  typeof reportError == "function"
    ? reportError
    : function (e) {
        console.error(e);
      };
function ca(e) {
  this._internalRoot = e;
}
hl.prototype.render = ca.prototype.render = function (e) {
  var t = this._internalRoot;
  if (t === null) throw Error(C(409));
  dl(e, t, null, null);
};
hl.prototype.unmount = ca.prototype.unmount = function () {
  var e = this._internalRoot;
  if (e !== null) {
    this._internalRoot = null;
    var t = e.containerInfo;
    jn(function () {
      dl(null, e, null, null);
    }),
      (t[_t] = null);
  }
};
function hl(e) {
  this._internalRoot = e;
}
hl.prototype.unstable_scheduleHydration = function (e) {
  if (e) {
    var t = Od();
    e = { blockedOn: null, target: e, priority: t };
    for (var n = 0; n < It.length && t !== 0 && t < It[n].priority; n++);
    It.splice(n, 0, e), n === 0 && Fd(e);
  }
};
function fa(e) {
  return !(!e || (e.nodeType !== 1 && e.nodeType !== 9 && e.nodeType !== 11));
}
function pl(e) {
  return !(
    !e ||
    (e.nodeType !== 1 &&
      e.nodeType !== 9 &&
      e.nodeType !== 11 &&
      (e.nodeType !== 8 || e.nodeValue !== " react-mount-point-unstable "))
  );
}
function Vc() {}
function My(e, t, n, r, i) {
  if (i) {
    if (typeof r == "function") {
      var o = r;
      r = function () {
        var a = Uo(l);
        o.call(a);
      };
    }
    var l = ip(t, r, e, 0, null, !1, !1, "", Vc);
    return (
      (e._reactRootContainer = l),
      (e[_t] = l.current),
      ci(e.nodeType === 8 ? e.parentNode : e),
      jn(),
      l
    );
  }
  for (; (i = e.lastChild); ) e.removeChild(i);
  if (typeof r == "function") {
    var s = r;
    r = function () {
      var a = Uo(u);
      s.call(a);
    };
  }
  var u = ua(e, 0, !1, null, null, !1, !1, "", Vc);
  return (
    (e._reactRootContainer = u),
    (e[_t] = u.current),
    ci(e.nodeType === 8 ? e.parentNode : e),
    jn(function () {
      dl(t, u, n, r);
    }),
    u
  );
}
function ml(e, t, n, r, i) {
  var o = n._reactRootContainer;
  if (o) {
    var l = o;
    if (typeof i == "function") {
      var s = i;
      i = function () {
        var u = Uo(l);
        s.call(u);
      };
    }
    dl(t, l, e, i);
  } else l = My(n, t, e, i, r);
  return Uo(l);
}
Dd = function (e) {
  switch (e.tag) {
    case 3:
      var t = e.stateNode;
      if (t.current.memoizedState.isDehydrated) {
        var n = Ur(t.pendingLanes);
        n !== 0 &&
          (Lu(t, n | 1), Oe(t, se()), !(V & 6) && ((dr = se() + 500), tn()));
      }
      break;
    case 13:
      jn(function () {
        var r = Lt(e, 1);
        if (r !== null) {
          var i = Ce();
          ct(r, e, 1, i);
        }
      }),
        aa(e, 1);
  }
};
Nu = function (e) {
  if (e.tag === 13) {
    var t = Lt(e, 134217728);
    if (t !== null) {
      var n = Ce();
      ct(t, e, 134217728, n);
    }
    aa(e, 134217728);
  }
};
zd = function (e) {
  if (e.tag === 13) {
    var t = Xt(e),
      n = Lt(e, t);
    if (n !== null) {
      var r = Ce();
      ct(n, e, t, r);
    }
    aa(e, t);
  }
};
Od = function () {
  return G;
};
Rd = function (e, t) {
  var n = G;
  try {
    return (G = e), t();
  } finally {
    G = n;
  }
};
Es = function (e, t, n) {
  switch (t) {
    case "input":
      if ((gs(e, n), (t = n.name), n.type === "radio" && t != null)) {
        for (n = e; n.parentNode; ) n = n.parentNode;
        for (
          n = n.querySelectorAll(
            "input[name=" + JSON.stringify("" + t) + '][type="radio"]'
          ),
            t = 0;
          t < n.length;
          t++
        ) {
          var r = n[t];
          if (r !== e && r.form === e.form) {
            var i = ol(r);
            if (!i) throw Error(C(90));
            pd(r), gs(r, i);
          }
        }
      }
      break;
    case "textarea":
      yd(e, n);
      break;
    case "select":
      (t = n.value), t != null && qn(e, !!n.multiple, t, !1);
  }
};
jd = ia;
Ed = jn;
var Ty = { usingClientEntryPoint: !1, Events: [_i, Vn, ol, Sd, kd, ia] },
  Nr = {
    findFiberByHostInstance: cn,
    bundleType: 0,
    version: "18.3.1",
    rendererPackageName: "react-dom",
  },
  _y = {
    bundleType: Nr.bundleType,
    version: Nr.version,
    rendererPackageName: Nr.rendererPackageName,
    rendererConfig: Nr.rendererConfig,
    overrideHookState: null,
    overrideHookStateDeletePath: null,
    overrideHookStateRenamePath: null,
    overrideProps: null,
    overridePropsDeletePath: null,
    overridePropsRenamePath: null,
    setErrorHandler: null,
    setSuspenseHandler: null,
    scheduleUpdate: null,
    currentDispatcherRef: Dt.ReactCurrentDispatcher,
    findHostInstanceByFiber: function (e) {
      return (e = Md(e)), e === null ? null : e.stateNode;
    },
    findFiberByHostInstance: Nr.findFiberByHostInstance || Cy,
    findHostInstancesForRefresh: null,
    scheduleRefresh: null,
    scheduleRoot: null,
    setRefreshHandler: null,
    getCurrentFiber: null,
    reconcilerVersion: "18.3.1-next-f1338f8080-20240426",
  };
if (typeof __REACT_DEVTOOLS_GLOBAL_HOOK__ < "u") {
  var Ji = __REACT_DEVTOOLS_GLOBAL_HOOK__;
  if (!Ji.isDisabled && Ji.supportsFiber)
    try {
      (tl = Ji.inject(_y)), (yt = Ji);
    } catch {}
}
Ye.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED = Ty;
Ye.createPortal = function (e, t) {
  var n = 2 < arguments.length && arguments[2] !== void 0 ? arguments[2] : null;
  if (!fa(t)) throw Error(C(200));
  return Py(e, t, null, n);
};
Ye.createRoot = function (e, t) {
  if (!fa(e)) throw Error(C(299));
  var n = !1,
    r = "",
    i = op;
  return (
    t != null &&
      (t.unstable_strictMode === !0 && (n = !0),
      t.identifierPrefix !== void 0 && (r = t.identifierPrefix),
      t.onRecoverableError !== void 0 && (i = t.onRecoverableError)),
    (t = ua(e, 1, !1, null, null, n, !1, r, i)),
    (e[_t] = t.current),
    ci(e.nodeType === 8 ? e.parentNode : e),
    new ca(t)
  );
};
Ye.findDOMNode = function (e) {
  if (e == null) return null;
  if (e.nodeType === 1) return e;
  var t = e._reactInternals;
  if (t === void 0)
    throw typeof e.render == "function"
      ? Error(C(188))
      : ((e = Object.keys(e).join(",")), Error(C(268, e)));
  return (e = Md(t)), (e = e === null ? null : e.stateNode), e;
};
Ye.flushSync = function (e) {
  return jn(e);
};
Ye.hydrate = function (e, t, n) {
  if (!pl(t)) throw Error(C(200));
  return ml(null, e, t, !0, n);
};
Ye.hydrateRoot = function (e, t, n) {
  if (!fa(e)) throw Error(C(405));
  var r = (n != null && n.hydratedSources) || null,
    i = !1,
    o = "",
    l = op;
  if (
    (n != null &&
      (n.unstable_strictMode === !0 && (i = !0),
      n.identifierPrefix !== void 0 && (o = n.identifierPrefix),
      n.onRecoverableError !== void 0 && (l = n.onRecoverableError)),
    (t = ip(t, null, e, 1, n ?? null, i, !1, o, l)),
    (e[_t] = t.current),
    ci(e),
    r)
  )
    for (e = 0; e < r.length; e++)
      (n = r[e]),
        (i = n._getVersion),
        (i = i(n._source)),
        t.mutableSourceEagerHydrationData == null
          ? (t.mutableSourceEagerHydrationData = [n, i])
          : t.mutableSourceEagerHydrationData.push(n, i);
  return new hl(t);
};
Ye.render = function (e, t, n) {
  if (!pl(t)) throw Error(C(200));
  return ml(null, e, t, !1, n);
};
Ye.unmountComponentAtNode = function (e) {
  if (!pl(e)) throw Error(C(40));
  return e._reactRootContainer
    ? (jn(function () {
        ml(null, null, e, !1, function () {
          (e._reactRootContainer = null), (e[_t] = null);
        });
      }),
      !0)
    : !1;
};
Ye.unstable_batchedUpdates = ia;
Ye.unstable_renderSubtreeIntoContainer = function (e, t, n, r) {
  if (!pl(n)) throw Error(C(200));
  if (e == null || e._reactInternals === void 0) throw Error(C(38));
  return ml(e, t, n, !1, r);
};
Ye.version = "18.3.1-next-f1338f8080-20240426";
function lp() {
  if (
    !(
      typeof __REACT_DEVTOOLS_GLOBAL_HOOK__ > "u" ||
      typeof __REACT_DEVTOOLS_GLOBAL_HOOK__.checkDCE != "function"
    )
  )
    try {
      __REACT_DEVTOOLS_GLOBAL_HOOK__.checkDCE(lp);
    } catch (e) {
      console.error(e);
    }
}
lp(), (ld.exports = Ye);
var Ly = ld.exports,
  Bc = Ly;
(cs.createRoot = Bc.createRoot), (cs.hydrateRoot = Bc.hydrateRoot);
const Yc = 0;
function Ny(e) {
  return c.jsx("div", {
    children: c.jsxs("table", {
      children: [
        c.jsxs("tr", {
          children: [
            c.jsx("th", { align: "left", children: " Wheel " }),
            c.jsx("th", { children: " Command [-]" }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "Left" }),
            c.jsx("td", {
              align: "center",
              children: e.robot.leftMotor.command.toFixed(Yc),
            }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "Right" }),
            c.jsx("td", {
              align: "center",
              children: e.robot.rightMotor.command.toFixed(Yc),
            }),
          ],
        }),
      ],
    }),
  });
}
function $y(e) {
  const t = e.robot.distances,
    n = e.distancePlot.labels;
  return c.jsx("div", {
    children: c.jsxs("table", {
      children: [
        c.jsxs("tr", {
          children: [
            c.jsx("th", { align: "left", children: " Sensor " }),
            c.jsx("th", { children: " Distance [mm]" }),
          ],
        }),
        t.map((r, i) =>
          c.jsxs(
            "tr",
            {
              children: [
                c.jsx("td", { children: n[i] }),
                c.jsx("td", { align: "center", children: r }),
              ],
            },
            i
          )
        ),
      ],
    }),
  });
}
const Dn = 2;
function Ay(e) {
  return c.jsx("div", {
    children: c.jsxs("table", {
      children: [
        c.jsxs("tr", {
          children: [
            c.jsx("th", { children: "Dimension" }),
            c.jsx("th", { children: "Linear [m/s^2] " }),
            c.jsx("th", { children: "Angular [rad/s]" }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "x" }),
            c.jsx("td", { children: e.acceleration.x.toFixed(Dn) }),
            c.jsx("td", { children: e.rotation.x.toFixed(Dn) }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "y" }),
            c.jsx("td", { children: e.acceleration.y.toFixed(Dn) }),
            c.jsx("td", { children: e.rotation.y.toFixed(Dn) }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "z" }),
            c.jsx("td", { children: e.acceleration.z.toFixed(Dn) }),
            c.jsx("td", { children: e.rotation.z.toFixed(Dn) }),
          ],
        }),
      ],
    }),
  });
}
function yo(e, t) {
  return e == null || t == null
    ? NaN
    : e < t
    ? -1
    : e > t
    ? 1
    : e >= t
    ? 0
    : NaN;
}
function Dy(e, t) {
  return e == null || t == null
    ? NaN
    : t < e
    ? -1
    : t > e
    ? 1
    : t >= e
    ? 0
    : NaN;
}
function da(e) {
  let t, n, r;
  e.length !== 2
    ? ((t = yo), (n = (s, u) => yo(e(s), u)), (r = (s, u) => e(s) - u))
    : ((t = e === yo || e === Dy ? e : zy), (n = e), (r = e));
  function i(s, u, a = 0, f = s.length) {
    if (a < f) {
      if (t(u, u) !== 0) return f;
      do {
        const d = (a + f) >>> 1;
        n(s[d], u) < 0 ? (a = d + 1) : (f = d);
      } while (a < f);
    }
    return a;
  }
  function o(s, u, a = 0, f = s.length) {
    if (a < f) {
      if (t(u, u) !== 0) return f;
      do {
        const d = (a + f) >>> 1;
        n(s[d], u) <= 0 ? (a = d + 1) : (f = d);
      } while (a < f);
    }
    return a;
  }
  function l(s, u, a = 0, f = s.length) {
    const d = i(s, u, a, f - 1);
    return d > a && r(s[d - 1], u) > -r(s[d], u) ? d - 1 : d;
  }
  return { left: i, center: l, right: o };
}
function zy() {
  return 0;
}
function Oy(e) {
  return e === null ? NaN : +e;
}
const Ry = da(yo),
  Fy = Ry.right;
da(Oy).center;
function ti(e, t) {
  let n, r;
  if (t === void 0)
    for (const i of e)
      i != null &&
        (n === void 0
          ? i >= i && (n = r = i)
          : (n > i && (n = i), r < i && (r = i)));
  else {
    let i = -1;
    for (let o of e)
      (o = t(o, ++i, e)) != null &&
        (n === void 0
          ? o >= o && (n = r = o)
          : (n > o && (n = o), r < o && (r = o)));
  }
  return [n, r];
}
class Qc extends Map {
  constructor(t, n = Hy) {
    if (
      (super(),
      Object.defineProperties(this, {
        _intern: { value: new Map() },
        _key: { value: n },
      }),
      t != null)
    )
      for (const [r, i] of t) this.set(r, i);
  }
  get(t) {
    return super.get(Gc(this, t));
  }
  has(t) {
    return super.has(Gc(this, t));
  }
  set(t, n) {
    return super.set(Iy(this, t), n);
  }
  delete(t) {
    return super.delete(Uy(this, t));
  }
}
function Gc({ _intern: e, _key: t }, n) {
  const r = t(n);
  return e.has(r) ? e.get(r) : n;
}
function Iy({ _intern: e, _key: t }, n) {
  const r = t(n);
  return e.has(r) ? e.get(r) : (e.set(r, n), n);
}
function Uy({ _intern: e, _key: t }, n) {
  const r = t(n);
  return e.has(r) && ((n = e.get(r)), e.delete(r)), n;
}
function Hy(e) {
  return e !== null && typeof e == "object" ? e.valueOf() : e;
}
const Wy = Math.sqrt(50),
  Vy = Math.sqrt(10),
  By = Math.sqrt(2);
function Ho(e, t, n) {
  const r = (t - e) / Math.max(0, n),
    i = Math.floor(Math.log10(r)),
    o = r / Math.pow(10, i),
    l = o >= Wy ? 10 : o >= Vy ? 5 : o >= By ? 2 : 1;
  let s, u, a;
  return (
    i < 0
      ? ((a = Math.pow(10, -i) / l),
        (s = Math.round(e * a)),
        (u = Math.round(t * a)),
        s / a < e && ++s,
        u / a > t && --u,
        (a = -a))
      : ((a = Math.pow(10, i) * l),
        (s = Math.round(e / a)),
        (u = Math.round(t / a)),
        s * a < e && ++s,
        u * a > t && --u),
    u < s && 0.5 <= n && n < 2 ? Ho(e, t, n * 2) : [s, u, a]
  );
}
function ru(e, t, n) {
  if (((t = +t), (e = +e), (n = +n), !(n > 0))) return [];
  if (e === t) return [e];
  const r = t < e,
    [i, o, l] = r ? Ho(t, e, n) : Ho(e, t, n);
  if (!(o >= i)) return [];
  const s = o - i + 1,
    u = new Array(s);
  if (r)
    if (l < 0) for (let a = 0; a < s; ++a) u[a] = (o - a) / -l;
    else for (let a = 0; a < s; ++a) u[a] = (o - a) * l;
  else if (l < 0) for (let a = 0; a < s; ++a) u[a] = (i + a) / -l;
  else for (let a = 0; a < s; ++a) u[a] = (i + a) * l;
  return u;
}
function iu(e, t, n) {
  return (t = +t), (e = +e), (n = +n), Ho(e, t, n)[2];
}
function ou(e, t, n) {
  (t = +t), (e = +e), (n = +n);
  const r = t < e,
    i = r ? iu(t, e, n) : iu(e, t, n);
  return (r ? -1 : 1) * (i < 0 ? 1 / -i : i);
}
function Yy(e, t) {
  let n;
  if (t === void 0)
    for (const r of e)
      r != null && (n < r || (n === void 0 && r >= r)) && (n = r);
  else {
    let r = -1;
    for (let i of e)
      (i = t(i, ++r, e)) != null &&
        (n < i || (n === void 0 && i >= i)) &&
        (n = i);
  }
  return n;
}
function Qy(e, t) {
  let n;
  if (t === void 0)
    for (const r of e)
      r != null && (n > r || (n === void 0 && r >= r)) && (n = r);
  else {
    let r = -1;
    for (let i of e)
      (i = t(i, ++r, e)) != null &&
        (n > i || (n === void 0 && i >= i)) &&
        (n = i);
  }
  return n;
}
function yl(e, t) {
  switch (arguments.length) {
    case 0:
      break;
    case 1:
      this.range(e);
      break;
    default:
      this.range(t).domain(e);
      break;
  }
  return this;
}
const bc = Symbol("implicit");
function ha() {
  var e = new Qc(),
    t = [],
    n = [],
    r = bc;
  function i(o) {
    let l = e.get(o);
    if (l === void 0) {
      if (r !== bc) return r;
      e.set(o, (l = t.push(o) - 1));
    }
    return n[l % n.length];
  }
  return (
    (i.domain = function (o) {
      if (!arguments.length) return t.slice();
      (t = []), (e = new Qc());
      for (const l of o) e.has(l) || e.set(l, t.push(l) - 1);
      return i;
    }),
    (i.range = function (o) {
      return arguments.length ? ((n = Array.from(o)), i) : n.slice();
    }),
    (i.unknown = function (o) {
      return arguments.length ? ((r = o), i) : r;
    }),
    (i.copy = function () {
      return ha(t, n).unknown(r);
    }),
    yl.apply(i, arguments),
    i
  );
}
function pa(e, t, n) {
  (e.prototype = t.prototype = n), (n.constructor = e);
}
function sp(e, t) {
  var n = Object.create(e.prototype);
  for (var r in t) n[r] = t[r];
  return n;
}
function Ni() {}
var vi = 0.7,
  Wo = 1 / vi,
  or = "\\s*([+-]?\\d+)\\s*",
  wi = "\\s*([+-]?(?:\\d*\\.)?\\d+(?:[eE][+-]?\\d+)?)\\s*",
  xt = "\\s*([+-]?(?:\\d*\\.)?\\d+(?:[eE][+-]?\\d+)?)%\\s*",
  Gy = /^#([0-9a-f]{3,8})$/,
  by = new RegExp(`^rgb\\(${or},${or},${or}\\)$`),
  Xy = new RegExp(`^rgb\\(${xt},${xt},${xt}\\)$`),
  Ky = new RegExp(`^rgba\\(${or},${or},${or},${wi}\\)$`),
  Zy = new RegExp(`^rgba\\(${xt},${xt},${xt},${wi}\\)$`),
  Jy = new RegExp(`^hsl\\(${wi},${xt},${xt}\\)$`),
  qy = new RegExp(`^hsla\\(${wi},${xt},${xt},${wi}\\)$`),
  Xc = {
    aliceblue: 15792383,
    antiquewhite: 16444375,
    aqua: 65535,
    aquamarine: 8388564,
    azure: 15794175,
    beige: 16119260,
    bisque: 16770244,
    black: 0,
    blanchedalmond: 16772045,
    blue: 255,
    blueviolet: 9055202,
    brown: 10824234,
    burlywood: 14596231,
    cadetblue: 6266528,
    chartreuse: 8388352,
    chocolate: 13789470,
    coral: 16744272,
    cornflowerblue: 6591981,
    cornsilk: 16775388,
    crimson: 14423100,
    cyan: 65535,
    darkblue: 139,
    darkcyan: 35723,
    darkgoldenrod: 12092939,
    darkgray: 11119017,
    darkgreen: 25600,
    darkgrey: 11119017,
    darkkhaki: 12433259,
    darkmagenta: 9109643,
    darkolivegreen: 5597999,
    darkorange: 16747520,
    darkorchid: 10040012,
    darkred: 9109504,
    darksalmon: 15308410,
    darkseagreen: 9419919,
    darkslateblue: 4734347,
    darkslategray: 3100495,
    darkslategrey: 3100495,
    darkturquoise: 52945,
    darkviolet: 9699539,
    deeppink: 16716947,
    deepskyblue: 49151,
    dimgray: 6908265,
    dimgrey: 6908265,
    dodgerblue: 2003199,
    firebrick: 11674146,
    floralwhite: 16775920,
    forestgreen: 2263842,
    fuchsia: 16711935,
    gainsboro: 14474460,
    ghostwhite: 16316671,
    gold: 16766720,
    goldenrod: 14329120,
    gray: 8421504,
    green: 32768,
    greenyellow: 11403055,
    grey: 8421504,
    honeydew: 15794160,
    hotpink: 16738740,
    indianred: 13458524,
    indigo: 4915330,
    ivory: 16777200,
    khaki: 15787660,
    lavender: 15132410,
    lavenderblush: 16773365,
    lawngreen: 8190976,
    lemonchiffon: 16775885,
    lightblue: 11393254,
    lightcoral: 15761536,
    lightcyan: 14745599,
    lightgoldenrodyellow: 16448210,
    lightgray: 13882323,
    lightgreen: 9498256,
    lightgrey: 13882323,
    lightpink: 16758465,
    lightsalmon: 16752762,
    lightseagreen: 2142890,
    lightskyblue: 8900346,
    lightslategray: 7833753,
    lightslategrey: 7833753,
    lightsteelblue: 11584734,
    lightyellow: 16777184,
    lime: 65280,
    limegreen: 3329330,
    linen: 16445670,
    magenta: 16711935,
    maroon: 8388608,
    mediumaquamarine: 6737322,
    mediumblue: 205,
    mediumorchid: 12211667,
    mediumpurple: 9662683,
    mediumseagreen: 3978097,
    mediumslateblue: 8087790,
    mediumspringgreen: 64154,
    mediumturquoise: 4772300,
    mediumvioletred: 13047173,
    midnightblue: 1644912,
    mintcream: 16121850,
    mistyrose: 16770273,
    moccasin: 16770229,
    navajowhite: 16768685,
    navy: 128,
    oldlace: 16643558,
    olive: 8421376,
    olivedrab: 7048739,
    orange: 16753920,
    orangered: 16729344,
    orchid: 14315734,
    palegoldenrod: 15657130,
    palegreen: 10025880,
    paleturquoise: 11529966,
    palevioletred: 14381203,
    papayawhip: 16773077,
    peachpuff: 16767673,
    peru: 13468991,
    pink: 16761035,
    plum: 14524637,
    powderblue: 11591910,
    purple: 8388736,
    rebeccapurple: 6697881,
    red: 16711680,
    rosybrown: 12357519,
    royalblue: 4286945,
    saddlebrown: 9127187,
    salmon: 16416882,
    sandybrown: 16032864,
    seagreen: 3050327,
    seashell: 16774638,
    sienna: 10506797,
    silver: 12632256,
    skyblue: 8900331,
    slateblue: 6970061,
    slategray: 7372944,
    slategrey: 7372944,
    snow: 16775930,
    springgreen: 65407,
    steelblue: 4620980,
    tan: 13808780,
    teal: 32896,
    thistle: 14204888,
    tomato: 16737095,
    turquoise: 4251856,
    violet: 15631086,
    wheat: 16113331,
    white: 16777215,
    whitesmoke: 16119285,
    yellow: 16776960,
    yellowgreen: 10145074,
  };
pa(Ni, Si, {
  copy(e) {
    return Object.assign(new this.constructor(), this, e);
  },
  displayable() {
    return this.rgb().displayable();
  },
  hex: Kc,
  formatHex: Kc,
  formatHex8: eg,
  formatHsl: tg,
  formatRgb: Zc,
  toString: Zc,
});
function Kc() {
  return this.rgb().formatHex();
}
function eg() {
  return this.rgb().formatHex8();
}
function tg() {
  return up(this).formatHsl();
}
function Zc() {
  return this.rgb().formatRgb();
}
function Si(e) {
  var t, n;
  return (
    (e = (e + "").trim().toLowerCase()),
    (t = Gy.exec(e))
      ? ((n = t[1].length),
        (t = parseInt(t[1], 16)),
        n === 6
          ? Jc(t)
          : n === 3
          ? new Ae(
              ((t >> 8) & 15) | ((t >> 4) & 240),
              ((t >> 4) & 15) | (t & 240),
              ((t & 15) << 4) | (t & 15),
              1
            )
          : n === 8
          ? qi(
              (t >> 24) & 255,
              (t >> 16) & 255,
              (t >> 8) & 255,
              (t & 255) / 255
            )
          : n === 4
          ? qi(
              ((t >> 12) & 15) | ((t >> 8) & 240),
              ((t >> 8) & 15) | ((t >> 4) & 240),
              ((t >> 4) & 15) | (t & 240),
              (((t & 15) << 4) | (t & 15)) / 255
            )
          : null)
      : (t = by.exec(e))
      ? new Ae(t[1], t[2], t[3], 1)
      : (t = Xy.exec(e))
      ? new Ae((t[1] * 255) / 100, (t[2] * 255) / 100, (t[3] * 255) / 100, 1)
      : (t = Ky.exec(e))
      ? qi(t[1], t[2], t[3], t[4])
      : (t = Zy.exec(e))
      ? qi((t[1] * 255) / 100, (t[2] * 255) / 100, (t[3] * 255) / 100, t[4])
      : (t = Jy.exec(e))
      ? tf(t[1], t[2] / 100, t[3] / 100, 1)
      : (t = qy.exec(e))
      ? tf(t[1], t[2] / 100, t[3] / 100, t[4])
      : Xc.hasOwnProperty(e)
      ? Jc(Xc[e])
      : e === "transparent"
      ? new Ae(NaN, NaN, NaN, 0)
      : null
  );
}
function Jc(e) {
  return new Ae((e >> 16) & 255, (e >> 8) & 255, e & 255, 1);
}
function qi(e, t, n, r) {
  return r <= 0 && (e = t = n = NaN), new Ae(e, t, n, r);
}
function ng(e) {
  return (
    e instanceof Ni || (e = Si(e)),
    e ? ((e = e.rgb()), new Ae(e.r, e.g, e.b, e.opacity)) : new Ae()
  );
}
function lu(e, t, n, r) {
  return arguments.length === 1 ? ng(e) : new Ae(e, t, n, r ?? 1);
}
function Ae(e, t, n, r) {
  (this.r = +e), (this.g = +t), (this.b = +n), (this.opacity = +r);
}
pa(
  Ae,
  lu,
  sp(Ni, {
    brighter(e) {
      return (
        (e = e == null ? Wo : Math.pow(Wo, e)),
        new Ae(this.r * e, this.g * e, this.b * e, this.opacity)
      );
    },
    darker(e) {
      return (
        (e = e == null ? vi : Math.pow(vi, e)),
        new Ae(this.r * e, this.g * e, this.b * e, this.opacity)
      );
    },
    rgb() {
      return this;
    },
    clamp() {
      return new Ae(yn(this.r), yn(this.g), yn(this.b), Vo(this.opacity));
    },
    displayable() {
      return (
        -0.5 <= this.r &&
        this.r < 255.5 &&
        -0.5 <= this.g &&
        this.g < 255.5 &&
        -0.5 <= this.b &&
        this.b < 255.5 &&
        0 <= this.opacity &&
        this.opacity <= 1
      );
    },
    hex: qc,
    formatHex: qc,
    formatHex8: rg,
    formatRgb: ef,
    toString: ef,
  })
);
function qc() {
  return `#${hn(this.r)}${hn(this.g)}${hn(this.b)}`;
}
function rg() {
  return `#${hn(this.r)}${hn(this.g)}${hn(this.b)}${hn(
    (isNaN(this.opacity) ? 1 : this.opacity) * 255
  )}`;
}
function ef() {
  const e = Vo(this.opacity);
  return `${e === 1 ? "rgb(" : "rgba("}${yn(this.r)}, ${yn(this.g)}, ${yn(
    this.b
  )}${e === 1 ? ")" : `, ${e})`}`;
}
function Vo(e) {
  return isNaN(e) ? 1 : Math.max(0, Math.min(1, e));
}
function yn(e) {
  return Math.max(0, Math.min(255, Math.round(e) || 0));
}
function hn(e) {
  return (e = yn(e)), (e < 16 ? "0" : "") + e.toString(16);
}
function tf(e, t, n, r) {
  return (
    r <= 0
      ? (e = t = n = NaN)
      : n <= 0 || n >= 1
      ? (e = t = NaN)
      : t <= 0 && (e = NaN),
    new st(e, t, n, r)
  );
}
function up(e) {
  if (e instanceof st) return new st(e.h, e.s, e.l, e.opacity);
  if ((e instanceof Ni || (e = Si(e)), !e)) return new st();
  if (e instanceof st) return e;
  e = e.rgb();
  var t = e.r / 255,
    n = e.g / 255,
    r = e.b / 255,
    i = Math.min(t, n, r),
    o = Math.max(t, n, r),
    l = NaN,
    s = o - i,
    u = (o + i) / 2;
  return (
    s
      ? (t === o
          ? (l = (n - r) / s + (n < r) * 6)
          : n === o
          ? (l = (r - t) / s + 2)
          : (l = (t - n) / s + 4),
        (s /= u < 0.5 ? o + i : 2 - o - i),
        (l *= 60))
      : (s = u > 0 && u < 1 ? 0 : l),
    new st(l, s, u, e.opacity)
  );
}
function ig(e, t, n, r) {
  return arguments.length === 1 ? up(e) : new st(e, t, n, r ?? 1);
}
function st(e, t, n, r) {
  (this.h = +e), (this.s = +t), (this.l = +n), (this.opacity = +r);
}
pa(
  st,
  ig,
  sp(Ni, {
    brighter(e) {
      return (
        (e = e == null ? Wo : Math.pow(Wo, e)),
        new st(this.h, this.s, this.l * e, this.opacity)
      );
    },
    darker(e) {
      return (
        (e = e == null ? vi : Math.pow(vi, e)),
        new st(this.h, this.s, this.l * e, this.opacity)
      );
    },
    rgb() {
      var e = (this.h % 360) + (this.h < 0) * 360,
        t = isNaN(e) || isNaN(this.s) ? 0 : this.s,
        n = this.l,
        r = n + (n < 0.5 ? n : 1 - n) * t,
        i = 2 * n - r;
      return new Ae(
        Xl(e >= 240 ? e - 240 : e + 120, i, r),
        Xl(e, i, r),
        Xl(e < 120 ? e + 240 : e - 120, i, r),
        this.opacity
      );
    },
    clamp() {
      return new st(nf(this.h), eo(this.s), eo(this.l), Vo(this.opacity));
    },
    displayable() {
      return (
        ((0 <= this.s && this.s <= 1) || isNaN(this.s)) &&
        0 <= this.l &&
        this.l <= 1 &&
        0 <= this.opacity &&
        this.opacity <= 1
      );
    },
    formatHsl() {
      const e = Vo(this.opacity);
      return `${e === 1 ? "hsl(" : "hsla("}${nf(this.h)}, ${
        eo(this.s) * 100
      }%, ${eo(this.l) * 100}%${e === 1 ? ")" : `, ${e})`}`;
    },
  })
);
function nf(e) {
  return (e = (e || 0) % 360), e < 0 ? e + 360 : e;
}
function eo(e) {
  return Math.max(0, Math.min(1, e || 0));
}
function Xl(e, t, n) {
  return (
    (e < 60
      ? t + ((n - t) * e) / 60
      : e < 180
      ? n
      : e < 240
      ? t + ((n - t) * (240 - e)) / 60
      : t) * 255
  );
}
const ma = (e) => () => e;
function og(e, t) {
  return function (n) {
    return e + n * t;
  };
}
function lg(e, t, n) {
  return (
    (e = Math.pow(e, n)),
    (t = Math.pow(t, n) - e),
    (n = 1 / n),
    function (r) {
      return Math.pow(e + r * t, n);
    }
  );
}
function sg(e) {
  return (e = +e) == 1
    ? ap
    : function (t, n) {
        return n - t ? lg(t, n, e) : ma(isNaN(t) ? n : t);
      };
}
function ap(e, t) {
  var n = t - e;
  return n ? og(e, n) : ma(isNaN(e) ? t : e);
}
const rf = (function e(t) {
  var n = sg(t);
  function r(i, o) {
    var l = n((i = lu(i)).r, (o = lu(o)).r),
      s = n(i.g, o.g),
      u = n(i.b, o.b),
      a = ap(i.opacity, o.opacity);
    return function (f) {
      return (
        (i.r = l(f)), (i.g = s(f)), (i.b = u(f)), (i.opacity = a(f)), i + ""
      );
    };
  }
  return (r.gamma = e), r;
})(1);
function ug(e, t) {
  t || (t = []);
  var n = e ? Math.min(t.length, e.length) : 0,
    r = t.slice(),
    i;
  return function (o) {
    for (i = 0; i < n; ++i) r[i] = e[i] * (1 - o) + t[i] * o;
    return r;
  };
}
function ag(e) {
  return ArrayBuffer.isView(e) && !(e instanceof DataView);
}
function cg(e, t) {
  var n = t ? t.length : 0,
    r = e ? Math.min(n, e.length) : 0,
    i = new Array(r),
    o = new Array(n),
    l;
  for (l = 0; l < r; ++l) i[l] = ya(e[l], t[l]);
  for (; l < n; ++l) o[l] = t[l];
  return function (s) {
    for (l = 0; l < r; ++l) o[l] = i[l](s);
    return o;
  };
}
function fg(e, t) {
  var n = new Date();
  return (
    (e = +e),
    (t = +t),
    function (r) {
      return n.setTime(e * (1 - r) + t * r), n;
    }
  );
}
function Bo(e, t) {
  return (
    (e = +e),
    (t = +t),
    function (n) {
      return e * (1 - n) + t * n;
    }
  );
}
function dg(e, t) {
  var n = {},
    r = {},
    i;
  (e === null || typeof e != "object") && (e = {}),
    (t === null || typeof t != "object") && (t = {});
  for (i in t) i in e ? (n[i] = ya(e[i], t[i])) : (r[i] = t[i]);
  return function (o) {
    for (i in n) r[i] = n[i](o);
    return r;
  };
}
var su = /[-+]?(?:\d+\.?\d*|\.?\d+)(?:[eE][-+]?\d+)?/g,
  Kl = new RegExp(su.source, "g");
function hg(e) {
  return function () {
    return e;
  };
}
function pg(e) {
  return function (t) {
    return e(t) + "";
  };
}
function mg(e, t) {
  var n = (su.lastIndex = Kl.lastIndex = 0),
    r,
    i,
    o,
    l = -1,
    s = [],
    u = [];
  for (e = e + "", t = t + ""; (r = su.exec(e)) && (i = Kl.exec(t)); )
    (o = i.index) > n &&
      ((o = t.slice(n, o)), s[l] ? (s[l] += o) : (s[++l] = o)),
      (r = r[0]) === (i = i[0])
        ? s[l]
          ? (s[l] += i)
          : (s[++l] = i)
        : ((s[++l] = null), u.push({ i: l, x: Bo(r, i) })),
      (n = Kl.lastIndex);
  return (
    n < t.length && ((o = t.slice(n)), s[l] ? (s[l] += o) : (s[++l] = o)),
    s.length < 2
      ? u[0]
        ? pg(u[0].x)
        : hg(t)
      : ((t = u.length),
        function (a) {
          for (var f = 0, d; f < t; ++f) s[(d = u[f]).i] = d.x(a);
          return s.join("");
        })
  );
}
function ya(e, t) {
  var n = typeof t,
    r;
  return t == null || n === "boolean"
    ? ma(t)
    : (n === "number"
        ? Bo
        : n === "string"
        ? (r = Si(t))
          ? ((t = r), rf)
          : mg
        : t instanceof Si
        ? rf
        : t instanceof Date
        ? fg
        : ag(t)
        ? ug
        : Array.isArray(t)
        ? cg
        : (typeof t.valueOf != "function" && typeof t.toString != "function") ||
          isNaN(t)
        ? dg
        : Bo)(e, t);
}
function yg(e, t) {
  return (
    (e = +e),
    (t = +t),
    function (n) {
      return Math.round(e * (1 - n) + t * n);
    }
  );
}
function gg(e) {
  return function () {
    return e;
  };
}
function xg(e) {
  return +e;
}
var of = [0, 1];
function Kn(e) {
  return e;
}
function uu(e, t) {
  return (t -= e = +e)
    ? function (n) {
        return (n - e) / t;
      }
    : gg(isNaN(t) ? NaN : 0.5);
}
function vg(e, t) {
  var n;
  return (
    e > t && ((n = e), (e = t), (t = n)),
    function (r) {
      return Math.max(e, Math.min(t, r));
    }
  );
}
function wg(e, t, n) {
  var r = e[0],
    i = e[1],
    o = t[0],
    l = t[1];
  return (
    i < r ? ((r = uu(i, r)), (o = n(l, o))) : ((r = uu(r, i)), (o = n(o, l))),
    function (s) {
      return o(r(s));
    }
  );
}
function Sg(e, t, n) {
  var r = Math.min(e.length, t.length) - 1,
    i = new Array(r),
    o = new Array(r),
    l = -1;
  for (
    e[r] < e[0] && ((e = e.slice().reverse()), (t = t.slice().reverse()));
    ++l < r;

  )
    (i[l] = uu(e[l], e[l + 1])), (o[l] = n(t[l], t[l + 1]));
  return function (s) {
    var u = Fy(e, s, 1, r) - 1;
    return o[u](i[u](s));
  };
}
function ga(e, t) {
  return t
    .domain(e.domain())
    .range(e.range())
    .interpolate(e.interpolate())
    .clamp(e.clamp())
    .unknown(e.unknown());
}
function cp() {
  var e = of,
    t = of,
    n = ya,
    r,
    i,
    o,
    l = Kn,
    s,
    u,
    a;
  function f() {
    var h = Math.min(e.length, t.length);
    return (
      l !== Kn && (l = vg(e[0], e[h - 1])),
      (s = h > 2 ? Sg : wg),
      (u = a = null),
      d
    );
  }
  function d(h) {
    return h == null || isNaN((h = +h))
      ? o
      : (u || (u = s(e.map(r), t, n)))(r(l(h)));
  }
  return (
    (d.invert = function (h) {
      return l(i((a || (a = s(t, e.map(r), Bo)))(h)));
    }),
    (d.domain = function (h) {
      return arguments.length ? ((e = Array.from(h, xg)), f()) : e.slice();
    }),
    (d.range = function (h) {
      return arguments.length ? ((t = Array.from(h)), f()) : t.slice();
    }),
    (d.rangeRound = function (h) {
      return (t = Array.from(h)), (n = yg), f();
    }),
    (d.clamp = function (h) {
      return arguments.length ? ((l = h ? !0 : Kn), f()) : l !== Kn;
    }),
    (d.interpolate = function (h) {
      return arguments.length ? ((n = h), f()) : n;
    }),
    (d.unknown = function (h) {
      return arguments.length ? ((o = h), d) : o;
    }),
    function (h, x) {
      return (r = h), (i = x), f();
    }
  );
}
function fp() {
  return cp()(Kn, Kn);
}
function kg(e) {
  return Math.abs((e = Math.round(e))) >= 1e21
    ? e.toLocaleString("en").replace(/,/g, "")
    : e.toString(10);
}
function Yo(e, t) {
  if (
    (n = (e = t ? e.toExponential(t - 1) : e.toExponential()).indexOf("e")) < 0
  )
    return null;
  var n,
    r = e.slice(0, n);
  return [r.length > 1 ? r[0] + r.slice(2) : r, +e.slice(n + 1)];
}
function hr(e) {
  return (e = Yo(Math.abs(e))), e ? e[1] : NaN;
}
function jg(e, t) {
  return function (n, r) {
    for (
      var i = n.length, o = [], l = 0, s = e[0], u = 0;
      i > 0 &&
      s > 0 &&
      (u + s + 1 > r && (s = Math.max(1, r - u)),
      o.push(n.substring((i -= s), i + s)),
      !((u += s + 1) > r));

    )
      s = e[(l = (l + 1) % e.length)];
    return o.reverse().join(t);
  };
}
function Eg(e) {
  return function (t) {
    return t.replace(/[0-9]/g, function (n) {
      return e[+n];
    });
  };
}
var Pg =
  /^(?:(.)?([<>=^]))?([+\-( ])?([$#])?(0)?(\d+)?(,)?(\.\d+)?(~)?([a-z%])?$/i;
function ki(e) {
  if (!(t = Pg.exec(e))) throw new Error("invalid format: " + e);
  var t;
  return new xa({
    fill: t[1],
    align: t[2],
    sign: t[3],
    symbol: t[4],
    zero: t[5],
    width: t[6],
    comma: t[7],
    precision: t[8] && t[8].slice(1),
    trim: t[9],
    type: t[10],
  });
}
ki.prototype = xa.prototype;
function xa(e) {
  (this.fill = e.fill === void 0 ? " " : e.fill + ""),
    (this.align = e.align === void 0 ? ">" : e.align + ""),
    (this.sign = e.sign === void 0 ? "-" : e.sign + ""),
    (this.symbol = e.symbol === void 0 ? "" : e.symbol + ""),
    (this.zero = !!e.zero),
    (this.width = e.width === void 0 ? void 0 : +e.width),
    (this.comma = !!e.comma),
    (this.precision = e.precision === void 0 ? void 0 : +e.precision),
    (this.trim = !!e.trim),
    (this.type = e.type === void 0 ? "" : e.type + "");
}
xa.prototype.toString = function () {
  return (
    this.fill +
    this.align +
    this.sign +
    this.symbol +
    (this.zero ? "0" : "") +
    (this.width === void 0 ? "" : Math.max(1, this.width | 0)) +
    (this.comma ? "," : "") +
    (this.precision === void 0 ? "" : "." + Math.max(0, this.precision | 0)) +
    (this.trim ? "~" : "") +
    this.type
  );
};
function Cg(e) {
  e: for (var t = e.length, n = 1, r = -1, i; n < t; ++n)
    switch (e[n]) {
      case ".":
        r = i = n;
        break;
      case "0":
        r === 0 && (r = n), (i = n);
        break;
      default:
        if (!+e[n]) break e;
        r > 0 && (r = 0);
        break;
    }
  return r > 0 ? e.slice(0, r) + e.slice(i + 1) : e;
}
var dp;
function Mg(e, t) {
  var n = Yo(e, t);
  if (!n) return e + "";
  var r = n[0],
    i = n[1],
    o = i - (dp = Math.max(-8, Math.min(8, Math.floor(i / 3))) * 3) + 1,
    l = r.length;
  return o === l
    ? r
    : o > l
    ? r + new Array(o - l + 1).join("0")
    : o > 0
    ? r.slice(0, o) + "." + r.slice(o)
    : "0." + new Array(1 - o).join("0") + Yo(e, Math.max(0, t + o - 1))[0];
}
function lf(e, t) {
  var n = Yo(e, t);
  if (!n) return e + "";
  var r = n[0],
    i = n[1];
  return i < 0
    ? "0." + new Array(-i).join("0") + r
    : r.length > i + 1
    ? r.slice(0, i + 1) + "." + r.slice(i + 1)
    : r + new Array(i - r.length + 2).join("0");
}
const sf = {
  "%": (e, t) => (e * 100).toFixed(t),
  b: (e) => Math.round(e).toString(2),
  c: (e) => e + "",
  d: kg,
  e: (e, t) => e.toExponential(t),
  f: (e, t) => e.toFixed(t),
  g: (e, t) => e.toPrecision(t),
  o: (e) => Math.round(e).toString(8),
  p: (e, t) => lf(e * 100, t),
  r: lf,
  s: Mg,
  X: (e) => Math.round(e).toString(16).toUpperCase(),
  x: (e) => Math.round(e).toString(16),
};
function uf(e) {
  return e;
}
var af = Array.prototype.map,
  cf = [
    "y",
    "z",
    "a",
    "f",
    "p",
    "n",
    "µ",
    "m",
    "",
    "k",
    "M",
    "G",
    "T",
    "P",
    "E",
    "Z",
    "Y",
  ];
function Tg(e) {
  var t =
      e.grouping === void 0 || e.thousands === void 0
        ? uf
        : jg(af.call(e.grouping, Number), e.thousands + ""),
    n = e.currency === void 0 ? "" : e.currency[0] + "",
    r = e.currency === void 0 ? "" : e.currency[1] + "",
    i = e.decimal === void 0 ? "." : e.decimal + "",
    o = e.numerals === void 0 ? uf : Eg(af.call(e.numerals, String)),
    l = e.percent === void 0 ? "%" : e.percent + "",
    s = e.minus === void 0 ? "−" : e.minus + "",
    u = e.nan === void 0 ? "NaN" : e.nan + "";
  function a(d) {
    d = ki(d);
    var h = d.fill,
      x = d.align,
      g = d.sign,
      v = d.symbol,
      j = d.zero,
      m = d.width,
      p = d.comma,
      y = d.precision,
      w = d.trim,
      S = d.type;
    S === "n"
      ? ((p = !0), (S = "g"))
      : sf[S] || (y === void 0 && (y = 12), (w = !0), (S = "g")),
      (j || (h === "0" && x === "=")) && ((j = !0), (h = "0"), (x = "="));
    var k =
        v === "$"
          ? n
          : v === "#" && /[boxX]/.test(S)
          ? "0" + S.toLowerCase()
          : "",
      E = v === "$" ? r : /[%p]/.test(S) ? l : "",
      P = sf[S],
      D = /[defgprs%]/.test(S);
    y =
      y === void 0
        ? 6
        : /[gprs]/.test(S)
        ? Math.max(1, Math.min(21, y))
        : Math.max(0, Math.min(20, y));
    function O(N) {
      var W = k,
        R = E,
        Q,
        K,
        b;
      if (S === "c") (R = P(N) + R), (N = "");
      else {
        N = +N;
        var le = N < 0 || 1 / N < 0;
        if (
          ((N = isNaN(N) ? u : P(Math.abs(N), y)),
          w && (N = Cg(N)),
          le && +N == 0 && g !== "+" && (le = !1),
          (W =
            (le ? (g === "(" ? g : s) : g === "-" || g === "(" ? "" : g) + W),
          (R =
            (S === "s" ? cf[8 + dp / 3] : "") +
            R +
            (le && g === "(" ? ")" : "")),
          D)
        ) {
          for (Q = -1, K = N.length; ++Q < K; )
            if (((b = N.charCodeAt(Q)), 48 > b || b > 57)) {
              (R = (b === 46 ? i + N.slice(Q + 1) : N.slice(Q)) + R),
                (N = N.slice(0, Q));
              break;
            }
        }
      }
      p && !j && (N = t(N, 1 / 0));
      var T = W.length + N.length + R.length,
        L = T < m ? new Array(m - T + 1).join(h) : "";
      switch (
        (p && j && ((N = t(L + N, L.length ? m - R.length : 1 / 0)), (L = "")),
        x)
      ) {
        case "<":
          N = W + N + R + L;
          break;
        case "=":
          N = W + L + N + R;
          break;
        case "^":
          N = L.slice(0, (T = L.length >> 1)) + W + N + R + L.slice(T);
          break;
        default:
          N = L + W + N + R;
          break;
      }
      return o(N);
    }
    return (
      (O.toString = function () {
        return d + "";
      }),
      O
    );
  }
  function f(d, h) {
    var x = a(((d = ki(d)), (d.type = "f"), d)),
      g = Math.max(-8, Math.min(8, Math.floor(hr(h) / 3))) * 3,
      v = Math.pow(10, -g),
      j = cf[8 + g / 3];
    return function (m) {
      return x(v * m) + j;
    };
  }
  return { format: a, formatPrefix: f };
}
var to, va, hp;
_g({ thousands: ",", grouping: [3], currency: ["$", ""] });
function _g(e) {
  return (to = Tg(e)), (va = to.format), (hp = to.formatPrefix), to;
}
function Lg(e) {
  return Math.max(0, -hr(Math.abs(e)));
}
function Ng(e, t) {
  return Math.max(
    0,
    Math.max(-8, Math.min(8, Math.floor(hr(t) / 3))) * 3 - hr(Math.abs(e))
  );
}
function $g(e, t) {
  return (
    (e = Math.abs(e)), (t = Math.abs(t) - e), Math.max(0, hr(t) - hr(e)) + 1
  );
}
function Ag(e, t, n, r) {
  var i = ou(e, t, n),
    o;
  switch (((r = ki(r ?? ",f")), r.type)) {
    case "s": {
      var l = Math.max(Math.abs(e), Math.abs(t));
      return (
        r.precision == null && !isNaN((o = Ng(i, l))) && (r.precision = o),
        hp(r, l)
      );
    }
    case "":
    case "e":
    case "g":
    case "p":
    case "r": {
      r.precision == null &&
        !isNaN((o = $g(i, Math.max(Math.abs(e), Math.abs(t))))) &&
        (r.precision = o - (r.type === "e"));
      break;
    }
    case "f":
    case "%": {
      r.precision == null &&
        !isNaN((o = Lg(i))) &&
        (r.precision = o - (r.type === "%") * 2);
      break;
    }
  }
  return va(r);
}
function Dg(e) {
  var t = e.domain;
  return (
    (e.ticks = function (n) {
      var r = t();
      return ru(r[0], r[r.length - 1], n ?? 10);
    }),
    (e.tickFormat = function (n, r) {
      var i = t();
      return Ag(i[0], i[i.length - 1], n ?? 10, r);
    }),
    (e.nice = function (n) {
      n == null && (n = 10);
      var r = t(),
        i = 0,
        o = r.length - 1,
        l = r[i],
        s = r[o],
        u,
        a,
        f = 10;
      for (
        s < l && ((a = l), (l = s), (s = a), (a = i), (i = o), (o = a));
        f-- > 0;

      ) {
        if (((a = iu(l, s, n)), a === u)) return (r[i] = l), (r[o] = s), t(r);
        if (a > 0) (l = Math.floor(l / a) * a), (s = Math.ceil(s / a) * a);
        else if (a < 0) (l = Math.ceil(l * a) / a), (s = Math.floor(s * a) / a);
        else break;
        u = a;
      }
      return e;
    }),
    e
  );
}
function pp() {
  var e = fp();
  return (
    (e.copy = function () {
      return ga(e, pp());
    }),
    yl.apply(e, arguments),
    Dg(e)
  );
}
function mp(e, t) {
  e = e.slice();
  var n = 0,
    r = e.length - 1,
    i = e[n],
    o = e[r],
    l;
  return (
    o < i && ((l = n), (n = r), (r = l), (l = i), (i = o), (o = l)),
    (e[n] = t.floor(i)),
    (e[r] = t.ceil(o)),
    e
  );
}
function ff(e) {
  return Math.log(e);
}
function df(e) {
  return Math.exp(e);
}
function zg(e) {
  return -Math.log(-e);
}
function Og(e) {
  return -Math.exp(-e);
}
function Rg(e) {
  return isFinite(e) ? +("1e" + e) : e < 0 ? 0 : e;
}
function Fg(e) {
  return e === 10 ? Rg : e === Math.E ? Math.exp : (t) => Math.pow(e, t);
}
function Ig(e) {
  return e === Math.E
    ? Math.log
    : (e === 10 && Math.log10) ||
        (e === 2 && Math.log2) ||
        ((e = Math.log(e)), (t) => Math.log(t) / e);
}
function hf(e) {
  return (t, n) => -e(-t, n);
}
function Ug(e) {
  const t = e(ff, df),
    n = t.domain;
  let r = 10,
    i,
    o;
  function l() {
    return (
      (i = Ig(r)),
      (o = Fg(r)),
      n()[0] < 0 ? ((i = hf(i)), (o = hf(o)), e(zg, Og)) : e(ff, df),
      t
    );
  }
  return (
    (t.base = function (s) {
      return arguments.length ? ((r = +s), l()) : r;
    }),
    (t.domain = function (s) {
      return arguments.length ? (n(s), l()) : n();
    }),
    (t.ticks = (s) => {
      const u = n();
      let a = u[0],
        f = u[u.length - 1];
      const d = f < a;
      d && ([a, f] = [f, a]);
      let h = i(a),
        x = i(f),
        g,
        v;
      const j = s == null ? 10 : +s;
      let m = [];
      if (!(r % 1) && x - h < j) {
        if (((h = Math.floor(h)), (x = Math.ceil(x)), a > 0)) {
          for (; h <= x; ++h)
            for (g = 1; g < r; ++g)
              if (((v = h < 0 ? g / o(-h) : g * o(h)), !(v < a))) {
                if (v > f) break;
                m.push(v);
              }
        } else
          for (; h <= x; ++h)
            for (g = r - 1; g >= 1; --g)
              if (((v = h > 0 ? g / o(-h) : g * o(h)), !(v < a))) {
                if (v > f) break;
                m.push(v);
              }
        m.length * 2 < j && (m = ru(a, f, j));
      } else m = ru(h, x, Math.min(x - h, j)).map(o);
      return d ? m.reverse() : m;
    }),
    (t.tickFormat = (s, u) => {
      if (
        (s == null && (s = 10),
        u == null && (u = r === 10 ? "s" : ","),
        typeof u != "function" &&
          (!(r % 1) && (u = ki(u)).precision == null && (u.trim = !0),
          (u = va(u))),
        s === 1 / 0)
      )
        return u;
      const a = Math.max(1, (r * s) / t.ticks().length);
      return (f) => {
        let d = f / o(Math.round(i(f)));
        return d * r < r - 0.5 && (d *= r), d <= a ? u(f) : "";
      };
    }),
    (t.nice = () =>
      n(
        mp(n(), {
          floor: (s) => o(Math.floor(i(s))),
          ceil: (s) => o(Math.ceil(i(s))),
        })
      )),
    t
  );
}
function yp() {
  const e = Ug(cp()).domain([1, 10]);
  return (e.copy = () => ga(e, yp()).base(e.base())), yl.apply(e, arguments), e;
}
const Zl = new Date(),
  Jl = new Date();
function de(e, t, n, r) {
  function i(o) {
    return e((o = arguments.length === 0 ? new Date() : new Date(+o))), o;
  }
  return (
    (i.floor = (o) => (e((o = new Date(+o))), o)),
    (i.ceil = (o) => (e((o = new Date(o - 1))), t(o, 1), e(o), o)),
    (i.round = (o) => {
      const l = i(o),
        s = i.ceil(o);
      return o - l < s - o ? l : s;
    }),
    (i.offset = (o, l) => (
      t((o = new Date(+o)), l == null ? 1 : Math.floor(l)), o
    )),
    (i.range = (o, l, s) => {
      const u = [];
      if (
        ((o = i.ceil(o)),
        (s = s == null ? 1 : Math.floor(s)),
        !(o < l) || !(s > 0))
      )
        return u;
      let a;
      do u.push((a = new Date(+o))), t(o, s), e(o);
      while (a < o && o < l);
      return u;
    }),
    (i.filter = (o) =>
      de(
        (l) => {
          if (l >= l) for (; e(l), !o(l); ) l.setTime(l - 1);
        },
        (l, s) => {
          if (l >= l)
            if (s < 0) for (; ++s <= 0; ) for (; t(l, -1), !o(l); );
            else for (; --s >= 0; ) for (; t(l, 1), !o(l); );
        }
      )),
    n &&
      ((i.count = (o, l) => (
        Zl.setTime(+o), Jl.setTime(+l), e(Zl), e(Jl), Math.floor(n(Zl, Jl))
      )),
      (i.every = (o) => (
        (o = Math.floor(o)),
        !isFinite(o) || !(o > 0)
          ? null
          : o > 1
          ? i.filter(r ? (l) => r(l) % o === 0 : (l) => i.count(0, l) % o === 0)
          : i
      ))),
    i
  );
}
const Qo = de(
  () => {},
  (e, t) => {
    e.setTime(+e + t);
  },
  (e, t) => t - e
);
Qo.every = (e) => (
  (e = Math.floor(e)),
  !isFinite(e) || !(e > 0)
    ? null
    : e > 1
    ? de(
        (t) => {
          t.setTime(Math.floor(t / e) * e);
        },
        (t, n) => {
          t.setTime(+t + n * e);
        },
        (t, n) => (n - t) / e
      )
    : Qo
);
Qo.range;
const Pt = 1e3,
  Je = Pt * 60,
  Ct = Je * 60,
  $t = Ct * 24,
  wa = $t * 7,
  pf = $t * 30,
  ql = $t * 365,
  Zn = de(
    (e) => {
      e.setTime(e - e.getMilliseconds());
    },
    (e, t) => {
      e.setTime(+e + t * Pt);
    },
    (e, t) => (t - e) / Pt,
    (e) => e.getUTCSeconds()
  );
Zn.range;
const Sa = de(
  (e) => {
    e.setTime(e - e.getMilliseconds() - e.getSeconds() * Pt);
  },
  (e, t) => {
    e.setTime(+e + t * Je);
  },
  (e, t) => (t - e) / Je,
  (e) => e.getMinutes()
);
Sa.range;
const Hg = de(
  (e) => {
    e.setUTCSeconds(0, 0);
  },
  (e, t) => {
    e.setTime(+e + t * Je);
  },
  (e, t) => (t - e) / Je,
  (e) => e.getUTCMinutes()
);
Hg.range;
const ka = de(
  (e) => {
    e.setTime(
      e - e.getMilliseconds() - e.getSeconds() * Pt - e.getMinutes() * Je
    );
  },
  (e, t) => {
    e.setTime(+e + t * Ct);
  },
  (e, t) => (t - e) / Ct,
  (e) => e.getHours()
);
ka.range;
const Wg = de(
  (e) => {
    e.setUTCMinutes(0, 0, 0);
  },
  (e, t) => {
    e.setTime(+e + t * Ct);
  },
  (e, t) => (t - e) / Ct,
  (e) => e.getUTCHours()
);
Wg.range;
const $i = de(
  (e) => e.setHours(0, 0, 0, 0),
  (e, t) => e.setDate(e.getDate() + t),
  (e, t) => (t - e - (t.getTimezoneOffset() - e.getTimezoneOffset()) * Je) / $t,
  (e) => e.getDate() - 1
);
$i.range;
const ja = de(
  (e) => {
    e.setUTCHours(0, 0, 0, 0);
  },
  (e, t) => {
    e.setUTCDate(e.getUTCDate() + t);
  },
  (e, t) => (t - e) / $t,
  (e) => e.getUTCDate() - 1
);
ja.range;
const Vg = de(
  (e) => {
    e.setUTCHours(0, 0, 0, 0);
  },
  (e, t) => {
    e.setUTCDate(e.getUTCDate() + t);
  },
  (e, t) => (t - e) / $t,
  (e) => Math.floor(e / $t)
);
Vg.range;
function Ln(e) {
  return de(
    (t) => {
      t.setDate(t.getDate() - ((t.getDay() + 7 - e) % 7)),
        t.setHours(0, 0, 0, 0);
    },
    (t, n) => {
      t.setDate(t.getDate() + n * 7);
    },
    (t, n) =>
      (n - t - (n.getTimezoneOffset() - t.getTimezoneOffset()) * Je) / wa
  );
}
const gl = Ln(0),
  Go = Ln(1),
  Bg = Ln(2),
  Yg = Ln(3),
  pr = Ln(4),
  Qg = Ln(5),
  Gg = Ln(6);
gl.range;
Go.range;
Bg.range;
Yg.range;
pr.range;
Qg.range;
Gg.range;
function Nn(e) {
  return de(
    (t) => {
      t.setUTCDate(t.getUTCDate() - ((t.getUTCDay() + 7 - e) % 7)),
        t.setUTCHours(0, 0, 0, 0);
    },
    (t, n) => {
      t.setUTCDate(t.getUTCDate() + n * 7);
    },
    (t, n) => (n - t) / wa
  );
}
const gp = Nn(0),
  bo = Nn(1),
  bg = Nn(2),
  Xg = Nn(3),
  mr = Nn(4),
  Kg = Nn(5),
  Zg = Nn(6);
gp.range;
bo.range;
bg.range;
Xg.range;
mr.range;
Kg.range;
Zg.range;
const Ea = de(
  (e) => {
    e.setDate(1), e.setHours(0, 0, 0, 0);
  },
  (e, t) => {
    e.setMonth(e.getMonth() + t);
  },
  (e, t) =>
    t.getMonth() - e.getMonth() + (t.getFullYear() - e.getFullYear()) * 12,
  (e) => e.getMonth()
);
Ea.range;
const Jg = de(
  (e) => {
    e.setUTCDate(1), e.setUTCHours(0, 0, 0, 0);
  },
  (e, t) => {
    e.setUTCMonth(e.getUTCMonth() + t);
  },
  (e, t) =>
    t.getUTCMonth() -
    e.getUTCMonth() +
    (t.getUTCFullYear() - e.getUTCFullYear()) * 12,
  (e) => e.getUTCMonth()
);
Jg.range;
const At = de(
  (e) => {
    e.setMonth(0, 1), e.setHours(0, 0, 0, 0);
  },
  (e, t) => {
    e.setFullYear(e.getFullYear() + t);
  },
  (e, t) => t.getFullYear() - e.getFullYear(),
  (e) => e.getFullYear()
);
At.every = (e) =>
  !isFinite((e = Math.floor(e))) || !(e > 0)
    ? null
    : de(
        (t) => {
          t.setFullYear(Math.floor(t.getFullYear() / e) * e),
            t.setMonth(0, 1),
            t.setHours(0, 0, 0, 0);
        },
        (t, n) => {
          t.setFullYear(t.getFullYear() + n * e);
        }
      );
At.range;
const En = de(
  (e) => {
    e.setUTCMonth(0, 1), e.setUTCHours(0, 0, 0, 0);
  },
  (e, t) => {
    e.setUTCFullYear(e.getUTCFullYear() + t);
  },
  (e, t) => t.getUTCFullYear() - e.getUTCFullYear(),
  (e) => e.getUTCFullYear()
);
En.every = (e) =>
  !isFinite((e = Math.floor(e))) || !(e > 0)
    ? null
    : de(
        (t) => {
          t.setUTCFullYear(Math.floor(t.getUTCFullYear() / e) * e),
            t.setUTCMonth(0, 1),
            t.setUTCHours(0, 0, 0, 0);
        },
        (t, n) => {
          t.setUTCFullYear(t.getUTCFullYear() + n * e);
        }
      );
En.range;
function qg(e, t, n, r, i, o) {
  const l = [
    [Zn, 1, Pt],
    [Zn, 5, 5 * Pt],
    [Zn, 15, 15 * Pt],
    [Zn, 30, 30 * Pt],
    [o, 1, Je],
    [o, 5, 5 * Je],
    [o, 15, 15 * Je],
    [o, 30, 30 * Je],
    [i, 1, Ct],
    [i, 3, 3 * Ct],
    [i, 6, 6 * Ct],
    [i, 12, 12 * Ct],
    [r, 1, $t],
    [r, 2, 2 * $t],
    [n, 1, wa],
    [t, 1, pf],
    [t, 3, 3 * pf],
    [e, 1, ql],
  ];
  function s(a, f, d) {
    const h = f < a;
    h && ([a, f] = [f, a]);
    const x = d && typeof d.range == "function" ? d : u(a, f, d),
      g = x ? x.range(a, +f + 1) : [];
    return h ? g.reverse() : g;
  }
  function u(a, f, d) {
    const h = Math.abs(f - a) / d,
      x = da(([, , j]) => j).right(l, h);
    if (x === l.length) return e.every(ou(a / ql, f / ql, d));
    if (x === 0) return Qo.every(Math.max(ou(a, f, d), 1));
    const [g, v] = l[h / l[x - 1][2] < l[x][2] / h ? x - 1 : x];
    return g.every(v);
  }
  return [s, u];
}
const [ex, tx] = qg(At, Ea, gl, $i, ka, Sa);
function es(e) {
  if (0 <= e.y && e.y < 100) {
    var t = new Date(-1, e.m, e.d, e.H, e.M, e.S, e.L);
    return t.setFullYear(e.y), t;
  }
  return new Date(e.y, e.m, e.d, e.H, e.M, e.S, e.L);
}
function ts(e) {
  if (0 <= e.y && e.y < 100) {
    var t = new Date(Date.UTC(-1, e.m, e.d, e.H, e.M, e.S, e.L));
    return t.setUTCFullYear(e.y), t;
  }
  return new Date(Date.UTC(e.y, e.m, e.d, e.H, e.M, e.S, e.L));
}
function $r(e, t, n) {
  return { y: e, m: t, d: n, H: 0, M: 0, S: 0, L: 0 };
}
function nx(e) {
  var t = e.dateTime,
    n = e.date,
    r = e.time,
    i = e.periods,
    o = e.days,
    l = e.shortDays,
    s = e.months,
    u = e.shortMonths,
    a = Ar(i),
    f = Dr(i),
    d = Ar(o),
    h = Dr(o),
    x = Ar(l),
    g = Dr(l),
    v = Ar(s),
    j = Dr(s),
    m = Ar(u),
    p = Dr(u),
    y = {
      a: le,
      A: T,
      b: L,
      B: z,
      c: null,
      d: wf,
      e: wf,
      f: Px,
      g: zx,
      G: Rx,
      H: kx,
      I: jx,
      j: Ex,
      L: xp,
      m: Cx,
      M: Mx,
      p: F,
      q: X,
      Q: jf,
      s: Ef,
      S: Tx,
      u: _x,
      U: Lx,
      V: Nx,
      w: $x,
      W: Ax,
      x: null,
      X: null,
      y: Dx,
      Y: Ox,
      Z: Fx,
      "%": kf,
    },
    w = {
      a: _e,
      A: Ge,
      b: nn,
      B: wt,
      c: null,
      d: Sf,
      e: Sf,
      f: Wx,
      g: Jx,
      G: e1,
      H: Ix,
      I: Ux,
      j: Hx,
      L: wp,
      m: Vx,
      M: Bx,
      p: $n,
      q: n0,
      Q: jf,
      s: Ef,
      S: Yx,
      u: Qx,
      U: Gx,
      V: bx,
      w: Xx,
      W: Kx,
      x: null,
      X: null,
      y: Zx,
      Y: qx,
      Z: t1,
      "%": kf,
    },
    S = {
      a: O,
      A: N,
      b: W,
      B: R,
      c: Q,
      d: xf,
      e: xf,
      f: xx,
      g: gf,
      G: yf,
      H: vf,
      I: vf,
      j: px,
      L: gx,
      m: hx,
      M: mx,
      p: D,
      q: dx,
      Q: wx,
      s: Sx,
      S: yx,
      u: sx,
      U: ux,
      V: ax,
      w: lx,
      W: cx,
      x: K,
      X: b,
      y: gf,
      Y: yf,
      Z: fx,
      "%": vx,
    };
  (y.x = k(n, y)),
    (y.X = k(r, y)),
    (y.c = k(t, y)),
    (w.x = k(n, w)),
    (w.X = k(r, w)),
    (w.c = k(t, w));
  function k(A, U) {
    return function (B) {
      var M = [],
        Le = -1,
        Z = 0,
        Re = A.length,
        Fe,
        rn,
        za;
      for (B instanceof Date || (B = new Date(+B)); ++Le < Re; )
        A.charCodeAt(Le) === 37 &&
          (M.push(A.slice(Z, Le)),
          (rn = mf[(Fe = A.charAt(++Le))]) != null
            ? (Fe = A.charAt(++Le))
            : (rn = Fe === "e" ? " " : "0"),
          (za = U[Fe]) && (Fe = za(B, rn)),
          M.push(Fe),
          (Z = Le + 1));
      return M.push(A.slice(Z, Le)), M.join("");
    };
  }
  function E(A, U) {
    return function (B) {
      var M = $r(1900, void 0, 1),
        Le = P(M, A, (B += ""), 0),
        Z,
        Re;
      if (Le != B.length) return null;
      if ("Q" in M) return new Date(M.Q);
      if ("s" in M) return new Date(M.s * 1e3 + ("L" in M ? M.L : 0));
      if (
        (U && !("Z" in M) && (M.Z = 0),
        "p" in M && (M.H = (M.H % 12) + M.p * 12),
        M.m === void 0 && (M.m = "q" in M ? M.q : 0),
        "V" in M)
      ) {
        if (M.V < 1 || M.V > 53) return null;
        "w" in M || (M.w = 1),
          "Z" in M
            ? ((Z = ts($r(M.y, 0, 1))),
              (Re = Z.getUTCDay()),
              (Z = Re > 4 || Re === 0 ? bo.ceil(Z) : bo(Z)),
              (Z = ja.offset(Z, (M.V - 1) * 7)),
              (M.y = Z.getUTCFullYear()),
              (M.m = Z.getUTCMonth()),
              (M.d = Z.getUTCDate() + ((M.w + 6) % 7)))
            : ((Z = es($r(M.y, 0, 1))),
              (Re = Z.getDay()),
              (Z = Re > 4 || Re === 0 ? Go.ceil(Z) : Go(Z)),
              (Z = $i.offset(Z, (M.V - 1) * 7)),
              (M.y = Z.getFullYear()),
              (M.m = Z.getMonth()),
              (M.d = Z.getDate() + ((M.w + 6) % 7)));
      } else
        ("W" in M || "U" in M) &&
          ("w" in M || (M.w = "u" in M ? M.u % 7 : "W" in M ? 1 : 0),
          (Re =
            "Z" in M
              ? ts($r(M.y, 0, 1)).getUTCDay()
              : es($r(M.y, 0, 1)).getDay()),
          (M.m = 0),
          (M.d =
            "W" in M
              ? ((M.w + 6) % 7) + M.W * 7 - ((Re + 5) % 7)
              : M.w + M.U * 7 - ((Re + 6) % 7)));
      return "Z" in M
        ? ((M.H += (M.Z / 100) | 0), (M.M += M.Z % 100), ts(M))
        : es(M);
    };
  }
  function P(A, U, B, M) {
    for (var Le = 0, Z = U.length, Re = B.length, Fe, rn; Le < Z; ) {
      if (M >= Re) return -1;
      if (((Fe = U.charCodeAt(Le++)), Fe === 37)) {
        if (
          ((Fe = U.charAt(Le++)),
          (rn = S[Fe in mf ? U.charAt(Le++) : Fe]),
          !rn || (M = rn(A, B, M)) < 0)
        )
          return -1;
      } else if (Fe != B.charCodeAt(M++)) return -1;
    }
    return M;
  }
  function D(A, U, B) {
    var M = a.exec(U.slice(B));
    return M ? ((A.p = f.get(M[0].toLowerCase())), B + M[0].length) : -1;
  }
  function O(A, U, B) {
    var M = x.exec(U.slice(B));
    return M ? ((A.w = g.get(M[0].toLowerCase())), B + M[0].length) : -1;
  }
  function N(A, U, B) {
    var M = d.exec(U.slice(B));
    return M ? ((A.w = h.get(M[0].toLowerCase())), B + M[0].length) : -1;
  }
  function W(A, U, B) {
    var M = m.exec(U.slice(B));
    return M ? ((A.m = p.get(M[0].toLowerCase())), B + M[0].length) : -1;
  }
  function R(A, U, B) {
    var M = v.exec(U.slice(B));
    return M ? ((A.m = j.get(M[0].toLowerCase())), B + M[0].length) : -1;
  }
  function Q(A, U, B) {
    return P(A, t, U, B);
  }
  function K(A, U, B) {
    return P(A, n, U, B);
  }
  function b(A, U, B) {
    return P(A, r, U, B);
  }
  function le(A) {
    return l[A.getDay()];
  }
  function T(A) {
    return o[A.getDay()];
  }
  function L(A) {
    return u[A.getMonth()];
  }
  function z(A) {
    return s[A.getMonth()];
  }
  function F(A) {
    return i[+(A.getHours() >= 12)];
  }
  function X(A) {
    return 1 + ~~(A.getMonth() / 3);
  }
  function _e(A) {
    return l[A.getUTCDay()];
  }
  function Ge(A) {
    return o[A.getUTCDay()];
  }
  function nn(A) {
    return u[A.getUTCMonth()];
  }
  function wt(A) {
    return s[A.getUTCMonth()];
  }
  function $n(A) {
    return i[+(A.getUTCHours() >= 12)];
  }
  function n0(A) {
    return 1 + ~~(A.getUTCMonth() / 3);
  }
  return {
    format: function (A) {
      var U = k((A += ""), y);
      return (
        (U.toString = function () {
          return A;
        }),
        U
      );
    },
    parse: function (A) {
      var U = E((A += ""), !1);
      return (
        (U.toString = function () {
          return A;
        }),
        U
      );
    },
    utcFormat: function (A) {
      var U = k((A += ""), w);
      return (
        (U.toString = function () {
          return A;
        }),
        U
      );
    },
    utcParse: function (A) {
      var U = E((A += ""), !0);
      return (
        (U.toString = function () {
          return A;
        }),
        U
      );
    },
  };
}
var mf = { "-": "", _: " ", 0: "0" },
  ye = /^\s*\d+/,
  rx = /^%/,
  ix = /[\\^$*+?|[\]().{}]/g;
function Y(e, t, n) {
  var r = e < 0 ? "-" : "",
    i = (r ? -e : e) + "",
    o = i.length;
  return r + (o < n ? new Array(n - o + 1).join(t) + i : i);
}
function ox(e) {
  return e.replace(ix, "\\$&");
}
function Ar(e) {
  return new RegExp("^(?:" + e.map(ox).join("|") + ")", "i");
}
function Dr(e) {
  return new Map(e.map((t, n) => [t.toLowerCase(), n]));
}
function lx(e, t, n) {
  var r = ye.exec(t.slice(n, n + 1));
  return r ? ((e.w = +r[0]), n + r[0].length) : -1;
}
function sx(e, t, n) {
  var r = ye.exec(t.slice(n, n + 1));
  return r ? ((e.u = +r[0]), n + r[0].length) : -1;
}
function ux(e, t, n) {
  var r = ye.exec(t.slice(n, n + 2));
  return r ? ((e.U = +r[0]), n + r[0].length) : -1;
}
function ax(e, t, n) {
  var r = ye.exec(t.slice(n, n + 2));
  return r ? ((e.V = +r[0]), n + r[0].length) : -1;
}
function cx(e, t, n) {
  var r = ye.exec(t.slice(n, n + 2));
  return r ? ((e.W = +r[0]), n + r[0].length) : -1;
}
function yf(e, t, n) {
  var r = ye.exec(t.slice(n, n + 4));
  return r ? ((e.y = +r[0]), n + r[0].length) : -1;
}
function gf(e, t, n) {
  var r = ye.exec(t.slice(n, n + 2));
  return r ? ((e.y = +r[0] + (+r[0] > 68 ? 1900 : 2e3)), n + r[0].length) : -1;
}
function fx(e, t, n) {
  var r = /^(Z)|([+-]\d\d)(?::?(\d\d))?/.exec(t.slice(n, n + 6));
  return r
    ? ((e.Z = r[1] ? 0 : -(r[2] + (r[3] || "00"))), n + r[0].length)
    : -1;
}
function dx(e, t, n) {
  var r = ye.exec(t.slice(n, n + 1));
  return r ? ((e.q = r[0] * 3 - 3), n + r[0].length) : -1;
}
function hx(e, t, n) {
  var r = ye.exec(t.slice(n, n + 2));
  return r ? ((e.m = r[0] - 1), n + r[0].length) : -1;
}
function xf(e, t, n) {
  var r = ye.exec(t.slice(n, n + 2));
  return r ? ((e.d = +r[0]), n + r[0].length) : -1;
}
function px(e, t, n) {
  var r = ye.exec(t.slice(n, n + 3));
  return r ? ((e.m = 0), (e.d = +r[0]), n + r[0].length) : -1;
}
function vf(e, t, n) {
  var r = ye.exec(t.slice(n, n + 2));
  return r ? ((e.H = +r[0]), n + r[0].length) : -1;
}
function mx(e, t, n) {
  var r = ye.exec(t.slice(n, n + 2));
  return r ? ((e.M = +r[0]), n + r[0].length) : -1;
}
function yx(e, t, n) {
  var r = ye.exec(t.slice(n, n + 2));
  return r ? ((e.S = +r[0]), n + r[0].length) : -1;
}
function gx(e, t, n) {
  var r = ye.exec(t.slice(n, n + 3));
  return r ? ((e.L = +r[0]), n + r[0].length) : -1;
}
function xx(e, t, n) {
  var r = ye.exec(t.slice(n, n + 6));
  return r ? ((e.L = Math.floor(r[0] / 1e3)), n + r[0].length) : -1;
}
function vx(e, t, n) {
  var r = rx.exec(t.slice(n, n + 1));
  return r ? n + r[0].length : -1;
}
function wx(e, t, n) {
  var r = ye.exec(t.slice(n));
  return r ? ((e.Q = +r[0]), n + r[0].length) : -1;
}
function Sx(e, t, n) {
  var r = ye.exec(t.slice(n));
  return r ? ((e.s = +r[0]), n + r[0].length) : -1;
}
function wf(e, t) {
  return Y(e.getDate(), t, 2);
}
function kx(e, t) {
  return Y(e.getHours(), t, 2);
}
function jx(e, t) {
  return Y(e.getHours() % 12 || 12, t, 2);
}
function Ex(e, t) {
  return Y(1 + $i.count(At(e), e), t, 3);
}
function xp(e, t) {
  return Y(e.getMilliseconds(), t, 3);
}
function Px(e, t) {
  return xp(e, t) + "000";
}
function Cx(e, t) {
  return Y(e.getMonth() + 1, t, 2);
}
function Mx(e, t) {
  return Y(e.getMinutes(), t, 2);
}
function Tx(e, t) {
  return Y(e.getSeconds(), t, 2);
}
function _x(e) {
  var t = e.getDay();
  return t === 0 ? 7 : t;
}
function Lx(e, t) {
  return Y(gl.count(At(e) - 1, e), t, 2);
}
function vp(e) {
  var t = e.getDay();
  return t >= 4 || t === 0 ? pr(e) : pr.ceil(e);
}
function Nx(e, t) {
  return (e = vp(e)), Y(pr.count(At(e), e) + (At(e).getDay() === 4), t, 2);
}
function $x(e) {
  return e.getDay();
}
function Ax(e, t) {
  return Y(Go.count(At(e) - 1, e), t, 2);
}
function Dx(e, t) {
  return Y(e.getFullYear() % 100, t, 2);
}
function zx(e, t) {
  return (e = vp(e)), Y(e.getFullYear() % 100, t, 2);
}
function Ox(e, t) {
  return Y(e.getFullYear() % 1e4, t, 4);
}
function Rx(e, t) {
  var n = e.getDay();
  return (
    (e = n >= 4 || n === 0 ? pr(e) : pr.ceil(e)), Y(e.getFullYear() % 1e4, t, 4)
  );
}
function Fx(e) {
  var t = e.getTimezoneOffset();
  return (
    (t > 0 ? "-" : ((t *= -1), "+")) +
    Y((t / 60) | 0, "0", 2) +
    Y(t % 60, "0", 2)
  );
}
function Sf(e, t) {
  return Y(e.getUTCDate(), t, 2);
}
function Ix(e, t) {
  return Y(e.getUTCHours(), t, 2);
}
function Ux(e, t) {
  return Y(e.getUTCHours() % 12 || 12, t, 2);
}
function Hx(e, t) {
  return Y(1 + ja.count(En(e), e), t, 3);
}
function wp(e, t) {
  return Y(e.getUTCMilliseconds(), t, 3);
}
function Wx(e, t) {
  return wp(e, t) + "000";
}
function Vx(e, t) {
  return Y(e.getUTCMonth() + 1, t, 2);
}
function Bx(e, t) {
  return Y(e.getUTCMinutes(), t, 2);
}
function Yx(e, t) {
  return Y(e.getUTCSeconds(), t, 2);
}
function Qx(e) {
  var t = e.getUTCDay();
  return t === 0 ? 7 : t;
}
function Gx(e, t) {
  return Y(gp.count(En(e) - 1, e), t, 2);
}
function Sp(e) {
  var t = e.getUTCDay();
  return t >= 4 || t === 0 ? mr(e) : mr.ceil(e);
}
function bx(e, t) {
  return (e = Sp(e)), Y(mr.count(En(e), e) + (En(e).getUTCDay() === 4), t, 2);
}
function Xx(e) {
  return e.getUTCDay();
}
function Kx(e, t) {
  return Y(bo.count(En(e) - 1, e), t, 2);
}
function Zx(e, t) {
  return Y(e.getUTCFullYear() % 100, t, 2);
}
function Jx(e, t) {
  return (e = Sp(e)), Y(e.getUTCFullYear() % 100, t, 2);
}
function qx(e, t) {
  return Y(e.getUTCFullYear() % 1e4, t, 4);
}
function e1(e, t) {
  var n = e.getUTCDay();
  return (
    (e = n >= 4 || n === 0 ? mr(e) : mr.ceil(e)),
    Y(e.getUTCFullYear() % 1e4, t, 4)
  );
}
function t1() {
  return "+0000";
}
function kf() {
  return "%";
}
function jf(e) {
  return +e;
}
function Ef(e) {
  return Math.floor(+e / 1e3);
}
var zn, kp;
n1({
  dateTime: "%x, %X",
  date: "%-m/%-d/%Y",
  time: "%-I:%M:%S %p",
  periods: ["AM", "PM"],
  days: [
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
  ],
  shortDays: ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"],
  months: [
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December",
  ],
  shortMonths: [
    "Jan",
    "Feb",
    "Mar",
    "Apr",
    "May",
    "Jun",
    "Jul",
    "Aug",
    "Sep",
    "Oct",
    "Nov",
    "Dec",
  ],
});
function n1(e) {
  return (
    (zn = nx(e)), (kp = zn.format), zn.parse, zn.utcFormat, zn.utcParse, zn
  );
}
function r1(e) {
  return new Date(e);
}
function i1(e) {
  return e instanceof Date ? +e : +new Date(+e);
}
function jp(e, t, n, r, i, o, l, s, u, a) {
  var f = fp(),
    d = f.invert,
    h = f.domain,
    x = a(".%L"),
    g = a(":%S"),
    v = a("%I:%M"),
    j = a("%I %p"),
    m = a("%a %d"),
    p = a("%b %d"),
    y = a("%B"),
    w = a("%Y");
  function S(k) {
    return (
      u(k) < k
        ? x
        : s(k) < k
        ? g
        : l(k) < k
        ? v
        : o(k) < k
        ? j
        : r(k) < k
        ? i(k) < k
          ? m
          : p
        : n(k) < k
        ? y
        : w
    )(k);
  }
  return (
    (f.invert = function (k) {
      return new Date(d(k));
    }),
    (f.domain = function (k) {
      return arguments.length ? h(Array.from(k, i1)) : h().map(r1);
    }),
    (f.ticks = function (k) {
      var E = h();
      return e(E[0], E[E.length - 1], k ?? 10);
    }),
    (f.tickFormat = function (k, E) {
      return E == null ? S : a(E);
    }),
    (f.nice = function (k) {
      var E = h();
      return (
        (!k || typeof k.range != "function") &&
          (k = t(E[0], E[E.length - 1], k ?? 10)),
        k ? h(mp(E, k)) : f
      );
    }),
    (f.copy = function () {
      return ga(f, jp(e, t, n, r, i, o, l, s, u, a));
    }),
    f
  );
}
function o1() {
  return yl.apply(
    jp(ex, tx, At, Ea, gl, $i, ka, Sa, Zn, kp).domain([
      new Date(2e3, 0, 1),
      new Date(2e3, 0, 2),
    ]),
    arguments
  );
}
function l1(e) {
  for (var t = (e.length / 6) | 0, n = new Array(t), r = 0; r < t; )
    n[r] = "#" + e.slice(r * 6, ++r * 6);
  return n;
}
const s1 = l1("e41a1c377eb84daf4a984ea3ff7f00ffff33a65628f781bf999999");
function ut(e) {
  for (
    var t = arguments.length, n = Array(t > 1 ? t - 1 : 0), r = 1;
    r < t;
    r++
  )
    n[r - 1] = arguments[r];
  throw Error(
    "[Immer] minified error nr: " +
      e +
      (n.length
        ? " " +
          n
            .map(function (i) {
              return "'" + i + "'";
            })
            .join(",")
        : "") +
      ". Find the full error at: https://bit.ly/3cXEKWf"
  );
}
function yr(e) {
  return !!e && !!e[Ve];
}
function Pn(e) {
  var t;
  return (
    !!e &&
    ((function (n) {
      if (!n || typeof n != "object") return !1;
      var r = Object.getPrototypeOf(n);
      if (r === null) return !0;
      var i = Object.hasOwnProperty.call(r, "constructor") && r.constructor;
      return (
        i === Object ||
        (typeof i == "function" && Function.toString.call(i) === m1)
      );
    })(e) ||
      Array.isArray(e) ||
      !!e[$f] ||
      !!(!((t = e.constructor) === null || t === void 0) && t[$f]) ||
      Pa(e) ||
      Ca(e))
  );
}
function ji(e, t, n) {
  n === void 0 && (n = !1),
    wr(e) === 0
      ? (n ? Object.keys : Na)(e).forEach(function (r) {
          (n && typeof r == "symbol") || t(r, e[r], e);
        })
      : e.forEach(function (r, i) {
          return t(i, r, e);
        });
}
function wr(e) {
  var t = e[Ve];
  return t
    ? t.i > 3
      ? t.i - 4
      : t.i
    : Array.isArray(e)
    ? 1
    : Pa(e)
    ? 2
    : Ca(e)
    ? 3
    : 0;
}
function au(e, t) {
  return wr(e) === 2 ? e.has(t) : Object.prototype.hasOwnProperty.call(e, t);
}
function u1(e, t) {
  return wr(e) === 2 ? e.get(t) : e[t];
}
function Ep(e, t, n) {
  var r = wr(e);
  r === 2 ? e.set(t, n) : r === 3 ? e.add(n) : (e[t] = n);
}
function a1(e, t) {
  return e === t ? e !== 0 || 1 / e == 1 / t : e != e && t != t;
}
function Pa(e) {
  return h1 && e instanceof Map;
}
function Ca(e) {
  return p1 && e instanceof Set;
}
function sn(e) {
  return e.o || e.t;
}
function Ma(e) {
  if (Array.isArray(e)) return Array.prototype.slice.call(e);
  var t = y1(e);
  delete t[Ve];
  for (var n = Na(t), r = 0; r < n.length; r++) {
    var i = n[r],
      o = t[i];
    o.writable === !1 && ((o.writable = !0), (o.configurable = !0)),
      (o.get || o.set) &&
        (t[i] = {
          configurable: !0,
          writable: !0,
          enumerable: o.enumerable,
          value: e[i],
        });
  }
  return Object.create(Object.getPrototypeOf(e), t);
}
function Ta(e, t) {
  return (
    t === void 0 && (t = !1),
    _a(e) ||
      yr(e) ||
      !Pn(e) ||
      (wr(e) > 1 && (e.set = e.add = e.clear = e.delete = c1),
      Object.freeze(e),
      t &&
        ji(
          e,
          function (n, r) {
            return Ta(r, !0);
          },
          !0
        )),
    e
  );
}
function c1() {
  ut(2);
}
function _a(e) {
  return e == null || typeof e != "object" || Object.isFrozen(e);
}
function vt(e) {
  var t = g1[e];
  return t || ut(18, e), t;
}
function Pf() {
  return Ei;
}
function ns(e, t) {
  t && (vt("Patches"), (e.u = []), (e.s = []), (e.v = t));
}
function Xo(e) {
  cu(e), e.p.forEach(f1), (e.p = null);
}
function cu(e) {
  e === Ei && (Ei = e.l);
}
function Cf(e) {
  return (Ei = { p: [], l: Ei, h: e, m: !0, _: 0 });
}
function f1(e) {
  var t = e[Ve];
  t.i === 0 || t.i === 1 ? t.j() : (t.g = !0);
}
function rs(e, t) {
  t._ = t.p.length;
  var n = t.p[0],
    r = e !== void 0 && e !== n;
  return (
    t.h.O || vt("ES5").S(t, e, r),
    r
      ? (n[Ve].P && (Xo(t), ut(4)),
        Pn(e) && ((e = Ko(t, e)), t.l || Zo(t, e)),
        t.u && vt("Patches").M(n[Ve].t, e, t.u, t.s))
      : (e = Ko(t, n, [])),
    Xo(t),
    t.u && t.v(t.u, t.s),
    e !== Pp ? e : void 0
  );
}
function Ko(e, t, n) {
  if (_a(t)) return t;
  var r = t[Ve];
  if (!r)
    return (
      ji(
        t,
        function (s, u) {
          return Mf(e, r, t, s, u, n);
        },
        !0
      ),
      t
    );
  if (r.A !== e) return t;
  if (!r.P) return Zo(e, r.t, !0), r.t;
  if (!r.I) {
    (r.I = !0), r.A._--;
    var i = r.i === 4 || r.i === 5 ? (r.o = Ma(r.k)) : r.o,
      o = i,
      l = !1;
    r.i === 3 && ((o = new Set(i)), i.clear(), (l = !0)),
      ji(o, function (s, u) {
        return Mf(e, r, i, s, u, n, l);
      }),
      Zo(e, i, !1),
      n && e.u && vt("Patches").N(r, n, e.u, e.s);
  }
  return r.o;
}
function Mf(e, t, n, r, i, o, l) {
  if (yr(i)) {
    var s = Ko(e, i, o && t && t.i !== 3 && !au(t.R, r) ? o.concat(r) : void 0);
    if ((Ep(n, r, s), !yr(s))) return;
    e.m = !1;
  } else l && n.add(i);
  if (Pn(i) && !_a(i)) {
    if (!e.h.D && e._ < 1) return;
    Ko(e, i), (t && t.A.l) || Zo(e, i);
  }
}
function Zo(e, t, n) {
  n === void 0 && (n = !1), !e.l && e.h.D && e.m && Ta(t, n);
}
function is(e, t) {
  var n = e[Ve];
  return (n ? sn(n) : e)[t];
}
function Tf(e, t) {
  if (t in e)
    for (var n = Object.getPrototypeOf(e); n; ) {
      var r = Object.getOwnPropertyDescriptor(n, t);
      if (r) return r;
      n = Object.getPrototypeOf(n);
    }
}
function fu(e) {
  e.P || ((e.P = !0), e.l && fu(e.l));
}
function os(e) {
  e.o || (e.o = Ma(e.t));
}
function du(e, t, n) {
  var r = Pa(t)
    ? vt("MapSet").F(t, n)
    : Ca(t)
    ? vt("MapSet").T(t, n)
    : e.O
    ? (function (i, o) {
        var l = Array.isArray(i),
          s = {
            i: l ? 1 : 0,
            A: o ? o.A : Pf(),
            P: !1,
            I: !1,
            R: {},
            l: o,
            t: i,
            k: null,
            o: null,
            j: null,
            C: !1,
          },
          u = s,
          a = hu;
        l && ((u = [s]), (a = Wr));
        var f = Proxy.revocable(u, a),
          d = f.revoke,
          h = f.proxy;
        return (s.k = h), (s.j = d), h;
      })(t, n)
    : vt("ES5").J(t, n);
  return (n ? n.A : Pf()).p.push(r), r;
}
function d1(e) {
  return (
    yr(e) || ut(22, e),
    (function t(n) {
      if (!Pn(n)) return n;
      var r,
        i = n[Ve],
        o = wr(n);
      if (i) {
        if (!i.P && (i.i < 4 || !vt("ES5").K(i))) return i.t;
        (i.I = !0), (r = _f(n, o)), (i.I = !1);
      } else r = _f(n, o);
      return (
        ji(r, function (l, s) {
          (i && u1(i.t, l) === s) || Ep(r, l, t(s));
        }),
        o === 3 ? new Set(r) : r
      );
    })(e)
  );
}
function _f(e, t) {
  switch (t) {
    case 2:
      return new Map(e);
    case 3:
      return Array.from(e);
  }
  return Ma(e);
}
var Lf,
  Ei,
  La = typeof Symbol < "u" && typeof Symbol("x") == "symbol",
  h1 = typeof Map < "u",
  p1 = typeof Set < "u",
  Nf = typeof Proxy < "u" && Proxy.revocable !== void 0 && typeof Reflect < "u",
  Pp = La
    ? Symbol.for("immer-nothing")
    : (((Lf = {})["immer-nothing"] = !0), Lf),
  $f = La ? Symbol.for("immer-draftable") : "__$immer_draftable",
  Ve = La ? Symbol.for("immer-state") : "__$immer_state",
  m1 = "" + Object.prototype.constructor,
  Na =
    typeof Reflect < "u" && Reflect.ownKeys
      ? Reflect.ownKeys
      : Object.getOwnPropertySymbols !== void 0
      ? function (e) {
          return Object.getOwnPropertyNames(e).concat(
            Object.getOwnPropertySymbols(e)
          );
        }
      : Object.getOwnPropertyNames,
  y1 =
    Object.getOwnPropertyDescriptors ||
    function (e) {
      var t = {};
      return (
        Na(e).forEach(function (n) {
          t[n] = Object.getOwnPropertyDescriptor(e, n);
        }),
        t
      );
    },
  g1 = {},
  hu = {
    get: function (e, t) {
      if (t === Ve) return e;
      var n = sn(e);
      if (!au(n, t))
        return (function (i, o, l) {
          var s,
            u = Tf(o, l);
          return u
            ? "value" in u
              ? u.value
              : (s = u.get) === null || s === void 0
              ? void 0
              : s.call(i.k)
            : void 0;
        })(e, n, t);
      var r = n[t];
      return e.I || !Pn(r)
        ? r
        : r === is(e.t, t)
        ? (os(e), (e.o[t] = du(e.A.h, r, e)))
        : r;
    },
    has: function (e, t) {
      return t in sn(e);
    },
    ownKeys: function (e) {
      return Reflect.ownKeys(sn(e));
    },
    set: function (e, t, n) {
      var r = Tf(sn(e), t);
      if (r != null && r.set) return r.set.call(e.k, n), !0;
      if (!e.P) {
        var i = is(sn(e), t),
          o = i == null ? void 0 : i[Ve];
        if (o && o.t === n) return (e.o[t] = n), (e.R[t] = !1), !0;
        if (a1(n, i) && (n !== void 0 || au(e.t, t))) return !0;
        os(e), fu(e);
      }
      return (
        (e.o[t] === n && (n !== void 0 || t in e.o)) ||
          (Number.isNaN(n) && Number.isNaN(e.o[t])) ||
          ((e.o[t] = n), (e.R[t] = !0)),
        !0
      );
    },
    deleteProperty: function (e, t) {
      return (
        is(e.t, t) !== void 0 || t in e.t
          ? ((e.R[t] = !1), os(e), fu(e))
          : delete e.R[t],
        e.o && delete e.o[t],
        !0
      );
    },
    getOwnPropertyDescriptor: function (e, t) {
      var n = sn(e),
        r = Reflect.getOwnPropertyDescriptor(n, t);
      return (
        r && {
          writable: !0,
          configurable: e.i !== 1 || t !== "length",
          enumerable: r.enumerable,
          value: n[t],
        }
      );
    },
    defineProperty: function () {
      ut(11);
    },
    getPrototypeOf: function (e) {
      return Object.getPrototypeOf(e.t);
    },
    setPrototypeOf: function () {
      ut(12);
    },
  },
  Wr = {};
ji(hu, function (e, t) {
  Wr[e] = function () {
    return (arguments[0] = arguments[0][0]), t.apply(this, arguments);
  };
}),
  (Wr.deleteProperty = function (e, t) {
    return Wr.set.call(this, e, t, void 0);
  }),
  (Wr.set = function (e, t, n) {
    return hu.set.call(this, e[0], t, n, e[0]);
  });
var x1 = (function () {
    function e(n) {
      var r = this;
      (this.O = Nf),
        (this.D = !0),
        (this.produce = function (i, o, l) {
          if (typeof i == "function" && typeof o != "function") {
            var s = o;
            o = i;
            var u = r;
            return function (v) {
              var j = this;
              v === void 0 && (v = s);
              for (
                var m = arguments.length, p = Array(m > 1 ? m - 1 : 0), y = 1;
                y < m;
                y++
              )
                p[y - 1] = arguments[y];
              return u.produce(v, function (w) {
                var S;
                return (S = o).call.apply(S, [j, w].concat(p));
              });
            };
          }
          var a;
          if (
            (typeof o != "function" && ut(6),
            l !== void 0 && typeof l != "function" && ut(7),
            Pn(i))
          ) {
            var f = Cf(r),
              d = du(r, i, void 0),
              h = !0;
            try {
              (a = o(d)), (h = !1);
            } finally {
              h ? Xo(f) : cu(f);
            }
            return typeof Promise < "u" && a instanceof Promise
              ? a.then(
                  function (v) {
                    return ns(f, l), rs(v, f);
                  },
                  function (v) {
                    throw (Xo(f), v);
                  }
                )
              : (ns(f, l), rs(a, f));
          }
          if (!i || typeof i != "object") {
            if (
              ((a = o(i)) === void 0 && (a = i),
              a === Pp && (a = void 0),
              r.D && Ta(a, !0),
              l)
            ) {
              var x = [],
                g = [];
              vt("Patches").M(i, a, x, g), l(x, g);
            }
            return a;
          }
          ut(21, i);
        }),
        (this.produceWithPatches = function (i, o) {
          if (typeof i == "function")
            return function (a) {
              for (
                var f = arguments.length, d = Array(f > 1 ? f - 1 : 0), h = 1;
                h < f;
                h++
              )
                d[h - 1] = arguments[h];
              return r.produceWithPatches(a, function (x) {
                return i.apply(void 0, [x].concat(d));
              });
            };
          var l,
            s,
            u = r.produce(i, o, function (a, f) {
              (l = a), (s = f);
            });
          return typeof Promise < "u" && u instanceof Promise
            ? u.then(function (a) {
                return [a, l, s];
              })
            : [u, l, s];
        }),
        typeof (n == null ? void 0 : n.useProxies) == "boolean" &&
          this.setUseProxies(n.useProxies),
        typeof (n == null ? void 0 : n.autoFreeze) == "boolean" &&
          this.setAutoFreeze(n.autoFreeze);
    }
    var t = e.prototype;
    return (
      (t.createDraft = function (n) {
        Pn(n) || ut(8), yr(n) && (n = d1(n));
        var r = Cf(this),
          i = du(this, n, void 0);
        return (i[Ve].C = !0), cu(r), i;
      }),
      (t.finishDraft = function (n, r) {
        var i = n && n[Ve],
          o = i.A;
        return ns(o, r), rs(void 0, o);
      }),
      (t.setAutoFreeze = function (n) {
        this.D = n;
      }),
      (t.setUseProxies = function (n) {
        n && !Nf && ut(20), (this.O = n);
      }),
      (t.applyPatches = function (n, r) {
        var i;
        for (i = r.length - 1; i >= 0; i--) {
          var o = r[i];
          if (o.path.length === 0 && o.op === "replace") {
            n = o.value;
            break;
          }
        }
        i > -1 && (r = r.slice(i + 1));
        var l = vt("Patches").$;
        return yr(n)
          ? l(n, r)
          : this.produce(n, function (s) {
              return l(s, r);
            });
      }),
      e
    );
  })(),
  Be = new x1(),
  Cp = Be.produce;
Be.produceWithPatches.bind(Be);
Be.setAutoFreeze.bind(Be);
Be.setUseProxies.bind(Be);
Be.applyPatches.bind(Be);
Be.createDraft.bind(Be);
Be.finishDraft.bind(Be);
const Af = { x: 0, y: 0, width: 0, height: 0 };
function pt() {
  const [e, t] = _.useState(Af),
    n = _.useRef(Af),
    r = _.useRef();
  return {
    ref: _.useCallback((o) => {
      if ((r.current && r.current(), o !== null)) {
        const l = new ResizeObserver(([s]) => {
          const u = s.target.getBBox(),
            a = n.current;
          if (
            a.x !== u.x ||
            a.y !== u.y ||
            a.width !== u.width ||
            a.height !== u.height
          ) {
            const f = { x: u.x, y: u.y, width: u.width, height: u.height };
            (n.current = f), t(f);
          }
        });
        l.observe(o),
          (r.current = () => {
            l.unobserve(o);
          });
      }
    }, []),
    ...e,
  };
}
function Df(e, t, n, r) {
  switch (t) {
    case "start":
      return e;
    case "end":
      return e - n;
    case "middle":
      return e - n / 2;
    case "none":
      return r;
    default:
      throw new Error(`Unkwnown alignment ${JSON.stringify(t)}`);
  }
}
function Ai(e) {
  const {
      x: t = 0,
      y: n = 0,
      verticalAlign: r = "start",
      horizontalAlign: i = "start",
      children: o,
      style: l = {},
    } = e,
    s = pt(),
    u = _.useMemo(() => Df(t - s.x, i, s.width || 0, t), [t, s.x, s.width, i]),
    a = _.useMemo(
      () => Df(n - s.y, r, s.height || 0, n),
      [n, s.y, s.height, r]
    );
  return c.jsx("g", {
    style: l,
    ref: s.ref,
    transform: `translate(${u}, ${a})`,
    children: o,
  });
}
function Jo(e, t) {
  var n, r;
  const i = document.createTextNode(e),
    o = document.createElementNS("http://www.w3.org/2000/svg", "text");
  o.setAttribute("class", "test"),
    o.appendChild(i),
    (n = t.current) === null || n === void 0 || n.appendChild(o);
  const l = o.getBBox();
  return (r = t.current) === null || r === void 0 || r.removeChild(o), l;
}
const v1 = "+1234567890";
function Mp(e, t, n, r) {
  const i = e.range();
  if (!i) throw new Error("Range needs to be specified");
  if (!e.domain()) throw new Error("Domain needs to be specified");
  const { minSpace: l = 8, tickFormat: s, setTicks: u } = r,
    a = Math.abs(i[0] - i[1]);
  _.useEffect(() => {
    if (n.current) {
      let f,
        d = [];
      if (t === "horizontal")
        for (;;) {
          d = e.ticks(f);
          const h = d.map(s);
          f = Math.min(d.length, f || 1 / 0);
          const { width: x } = Jo(h.join(""), n);
          if (x + (d.length - 1) * l > a && f > 1) f = f - 1;
          else break;
        }
      else {
        const { height: h } = Jo(v1, n);
        for (
          ;
          (d = e.ticks(f)),
            (f = Math.min(d.length, f || 1 / 0)),
            d.length * (h + l) - l > a && f > 1;

        )
          f = f - 1;
      }
      u(d.map((h) => ({ label: s(h), position: e(h), value: h })));
    }
  }, [a, t, l, n, e, u, s]);
}
function w1(e, t, n, r = {}) {
  const [i, o] = _.useState([]),
    { tickFormat: l = String } = r;
  return Mp(e, t, n, { ...r, tickFormat: l, setTicks: o }), i;
}
const S1 = "+1234567890";
function pu(e) {
  const t = e / Math.pow(10, Math.round(Math.log10(e)));
  return Math.floor(t < 1 ? t * 10 : t);
}
function k1(e, t, n, r, i) {
  const o = e.filter((f) => pu(f) === 1).map(t),
    l = Math.abs(o[0] - o[1]),
    s = (r + i) / l,
    u = s >= 1 ? Math.ceil(s) : 1;
  let a = 0;
  return e.map((f) => {
    const d = t(f);
    let h = "";
    return (
      pu(f) === 1 && ((h = a === 0 ? n(f) : ""), (a = (a + 1) % u)),
      { label: h, position: d, value: f }
    );
  });
}
function j1(e, t, n, r = {}) {
  const [i, o] = _.useState(40);
  if (!e.range()) throw new Error("Range needs to be specified");
  const s = e.domain();
  if (!s) throw new Error("Domain needs to be specified");
  const { minSpace: u = 8 } = r,
    a = r == null ? void 0 : r.tickFormat,
    f = _.useCallback((h) => (a ? a(h) : String(h)), [a]),
    d = _.useMemo(() => e.ticks(), [e]);
  return (
    _.useEffect(() => {
      if (n.current)
        if (t === "horizontal") {
          const h = d
              .filter((g) => pu(g) === 1)
              .map(f)
              .reduce((g, v) => (g.length < v.length ? v : g), ""),
            { width: x } = Jo(h, n);
          o(Math.ceil(x));
        } else {
          const { height: h } = Jo(S1, n);
          o(Math.ceil(h));
        }
    }, [t, s, f, n, d]),
    k1(d, e, f, i, u)
  );
}
function E1(e, t, n, r) {
  const { tickFormat: i = e.tickFormat() } = r,
    [o, l] = _.useState([]);
  return Mp(e, t, n, { ...r, setTicks: l, tickFormat: i }), o;
}
const Tp = _.createContext(null);
function Sr() {
  const e = _.useContext(Tp);
  if (!e) throw new Error("context was not initialized");
  return e;
}
const P1 = Cp((e, t) => {
    switch (t.type) {
      case "ADD_LEGEND_LABEL": {
        const { shape: n, ...r } = t.payload,
          i = e.labels.findIndex(({ id: o }) => r.id === o);
        if (i < 0) e.labels.push({ ...r, shape: n, isVisible: !0 });
        else {
          const o = e.labels[i].isVisible;
          e.labels[i] = { ...r, isVisible: o, shape: n };
        }
        return;
      }
      case "REMOVE_LEGEND_LABEL": {
        const { id: n } = t.payload,
          r = e.labels.findIndex((i) => i.id === n);
        r !== -1 && e.labels.splice(r, 1);
        return;
      }
      case "TOGGLE_VISIBILITY": {
        const { id: n } = t.payload,
          r = e.labels.findIndex((i) => i.id === n);
        r !== -1 && (e.labels[r].isVisible = !e.labels[r].isVisible);
        return;
      }
      default:
        throw new Error("unreachable");
    }
  }),
  C1 = { labels: [] },
  M1 = (e) => {
    const [t, n] = _.useReducer(P1, C1),
      r = _.useMemo(() => [t, n], [t, n]);
    return c.jsx(Tp.Provider, { value: r, children: e.children });
  };
let T1 = 1;
function _1() {
  return ++T1;
}
function kr(e, t) {
  return _.useMemo(() => e || `${t}-${_1()}`, [e, t]);
}
const an = ["top", "bottom"],
  Ft = ["left", "right"];
function L1(e, t, n) {
  if (
    (an.includes(e) && !an.includes(t)) ||
    (Ft.includes(e) && !Ft.includes(t))
  )
    throw new Error(`The positions are ortogonal for ${n}`);
}
function nt(e, t, n, r = {}) {
  const { onlyOrthogonal: i = !1 } = r,
    o = e[t],
    l = e[n];
  if (!o || !l) return [void 0, void 0];
  if (
    i &&
    ((an.includes(o.position) && an.includes(l.position)) ||
      (Ft.includes(l.position) && Ft.includes(o.position)))
  )
    throw new Error(`The axis ${t} and ${n} are not orthogonal`);
  if (
    !i &&
    (an.includes(o.position)
      ? !Ft.includes(l.position)
      : Ft.includes(o.position))
  )
    throw Ft.includes(o.position) || an.includes(l.position)
      ? new Error(
          `The axis ${t} should be ${an.join(" ")} and ${n} should be ${Ft.join(
            " "
          )}`
        )
      : new Error(`The axis ${t} and ${n} are not orthogonal`);
  return [o.scale, l.scale];
}
function gn(e, t, n, r, i) {
  let o = { ...e };
  for (const l in t)
    typeof t[l] == "function" ? (o[l] = t[l](n, r, i)) : (o[l] = t[l]);
  return o;
}
function N1(e, t, n, r) {
  let i;
  return typeof e == "function" ? (i = e(t, n, r)) : (i = e), i;
}
function $1(e, t, n, r) {
  let i;
  return typeof e == "function" ? (i = e(t, n, r)) : (i = e), i;
}
function zf(e) {
  return typeof e == "number"
    ? [e, e]
    : Array.isArray(e) && e.length >= 2
    ? e
    : null;
}
function ls(e, t, n) {
  let r = { index: 0, distance: Number.POSITIVE_INFINITY };
  for (let i = 0; i < e.length; i++) {
    const o = n(e[i], t);
    o < r.distance && ((r.index = i), (r.distance = o));
  }
  return e[r.index];
}
function Vr(e) {
  return typeof e > "u" || typeof e == "number" ? e : e.getTime();
}
function Of(e, t) {
  return { x: (e.x + t.x) / 2, y: (e.y + t.y) / 2 };
}
function A1(e, t) {
  switch (t.type) {
    case "addSeries": {
      e.series.push(t.payload);
      break;
    }
    case "removeSeries": {
      const { id: n } = t.payload,
        r = e.series.filter((i) => i.id !== n);
      e.series = r;
      break;
    }
    case "addAxis": {
      const { id: n, position: r, ...i } = t.payload,
        o = e.axes[n];
      o
        ? (L1(o.position, r, n), (e.axes[n] = { ...o, position: r, ...i }))
        : (e.axes[n] = { id: n, position: r, ...i });
      break;
    }
    case "removeAxis": {
      const { id: n } = t.payload;
      delete e.axes[n];
      break;
    }
    case "addHeading": {
      e.headingPosition = t.payload.position;
      break;
    }
    case "removeHeading": {
      e.headingPosition = null;
      break;
    }
    case "addLegend": {
      (e.legendPosition = t.payload.position),
        (e.legendMargin = t.payload.margin);
      break;
    }
    case "removeLegend": {
      (e.legendPosition = null), (e.legendMargin = 0);
      break;
    }
    default:
      throw new Error(`Unknown reducer type ${t.type}`);
  }
}
const _p = _.createContext({
    width: 0,
    height: 0,
    plotWidth: 0,
    plotHeight: 0,
    colorScaler: ha(),
    axisContext: {},
  }),
  Lp = _.createContext(() => {});
function ue() {
  const e = _.useContext(_p);
  if (!e) throw new Error("usePlotContext called outside of Plot context");
  return e;
}
function Di() {
  const e = _.useContext(Lp);
  if (!e)
    throw new Error("usePlotDispatchContext called outside of Plot context");
  return e;
}
function D1(e, t, { plotWidth: n, plotHeight: r }) {
  return _.useMemo(() => {
    const o = {};
    for (const l in e.axes) {
      const s = e.axes[l],
        u = t[l],
        a = ["top", "bottom"].includes(s.position),
        f = a ? "x" : "y";
      let d = !1,
        h;
      (u == null ? void 0 : u.min) != null
        ? ((h = u.min), (d = !0))
        : s.min != null
        ? ((h = s.min), (d = !0))
        : (h = Qy(
            e.series.filter((w) => f !== "x" || !w.id.startsWith("~")),
            (w) => (w[f].axisId === l ? w[f].min : void 0)
          ));
      let x = !1,
        g;
      if (
        ((u == null ? void 0 : u.max) != null
          ? ((g = u.max), (x = !0))
          : s.max != null
          ? ((g = s.max), (x = !0))
          : (g = Yy(
              e.series.filter((w) => f !== "x" || !w.id.startsWith("~")),
              (w) => (w[f].axisId === l ? w[f].max : void 0)
            )),
        h === void 0 || g === void 0)
      )
        return {};
      if (h > g) throw new Error(`${l}: min (${h}) is bigger than max (${g})`);
      const v = a ? n : r,
        j = z1(s, g - h, v, d, x),
        m = a ? [0, n] : [r, 0],
        p = [h - j.min, g + j.max],
        y = function (S) {
          return S < p[0] ? p[0] : S > p[1] ? p[1] : S;
        };
      switch (s.scale) {
        case "log": {
          o[l] = {
            type: s.scale,
            position: s.position,
            tickLabelFormat: s.tickLabelFormat,
            scale: yp()
              .domain(p)
              .range(s.flip ? m.reverse() : m),
            domain: p,
            clampInDomain: y,
          };
          break;
        }
        case "time": {
          o[l] = {
            type: s.scale,
            position: s.position,
            tickLabelFormat: s.tickLabelFormat,
            scale: o1()
              .domain(p)
              .range(s.flip ? m.reverse() : m),
            domain: p,
            clampInDomain: y,
          };
          break;
        }
        case "linear": {
          o[l] = {
            type: "linear",
            position: s.position,
            tickLabelFormat: s.tickLabelFormat,
            scale: pp()
              .domain(p)
              .range(s.flip ? m.reverse() : m),
            domain: p,
            clampInDomain: y,
          };
          break;
        }
        default:
          throw new Error("unreachable");
      }
    }
    return o;
  }, [e.axes, e.series, t, n, r]);
}
function z1(e, t, n, r, i) {
  const { paddingStart: o, paddingEnd: l } = e;
  if (r && i) return { min: 0, max: 0 };
  if (i) return { min: ss(o, 0, t, n).start, max: 0 };
  if (r) return { min: 0, max: ss(0, l, t, n).end };
  {
    const s = ss(o, l, t, n);
    return { min: s.start, max: s.end };
  }
}
function ss(e, t, n, r) {
  let i = 0,
    o = 0,
    l = n;
  typeof e == "number" && ((l += e), (i = e)),
    typeof t == "number" && ((l += t), (o = t));
  let s = 0,
    u = 0;
  typeof e == "string" && (s = Rf(e, r) / r),
    typeof t == "string" && (u = Rf(t, r) / r);
  const a = s + u;
  if (a !== 0) {
    const f = (a * l) / (1 - a);
    (i = (s / a) * f), (o = (u / a) * f);
  }
  return { start: i, end: o };
}
function Rf(e, t) {
  return e.endsWith("%") ? (Number(e.slice(0, -1)) / 100) * t : Number(e);
}
function xl(e) {
  const t = _.createContext(new Map());
  function n(i) {
    const o = _.useContext(t);
    return o.has(i) ? o.get(i) : e;
  }
  function r(i) {
    const { id: o, value: l, children: s } = i,
      u = _.useContext(t),
      a = _.useMemo(() => {
        const f = new Map(u);
        return f.set(o, l), f;
      }, [u, o, l]);
    return c.jsx(t.Provider, { value: a, children: s });
  }
  return { useNestedContext: n, NestedContextProvider: r };
}
const O1 = { axes: {} },
  R1 = xl(O1);
function F1(e) {
  return R1.useNestedContext(e == null ? void 0 : e.controllerId);
}
xl(null);
xl(null);
const I1 = xl(null);
function U1(e) {
  return I1.useNestedContext(e == null ? void 0 : e.controllerId);
}
function H1(e, t) {
  let n = 0;
  for (let r = 0; r < e.length; r++) n += (e[r] - t[r]) * (e[r] - t[r]);
  return n;
}
function Br(e, t) {
  return Math.sqrt(H1(e, t));
}
function Cn(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = ue(),
    { x: i, y: o, xAxis: l, yAxis: s } = e,
    [u, a] = nt(t, l, s, { onlyOrthogonal: !0 });
  return { x: je(i, n, u), y: je(o, r, a) };
}
function Np(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = ue(),
    { points: i, xAxis: o, yAxis: l } = e,
    [s, u] = nt(t, o, l, { onlyOrthogonal: !0 });
  return i.map((a) => `${je(a.x, n, s)},${je(a.y, r, u)}`).join(" ");
}
function W1(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = ue(),
    { x1: i, y1: o, x2: l, y2: s, xAxis: u, yAxis: a } = e,
    [f, d] = nt(t, u, a, { onlyOrthogonal: !0 });
  return {
    x: Ff(i, l, n, f),
    y: Ff(o, s, r, d),
    width: Uf(i, l, n, f),
    height: Uf(o, s, r, d),
  };
}
function $p(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = ue(),
    { cx: i, cy: o, rx: l, ry: s, xAxis: u, yAxis: a } = e,
    [f, d] = nt(t, u, a, { onlyOrthogonal: !0 });
  return { cx: je(i, n, f), cy: je(o, r, d), rx: Pi(l, n, f), ry: Pi(s, r, d) };
}
function V1(e) {
  var t;
  const { axisContext: n, plotWidth: r, plotHeight: i } = ue(),
    {
      min: o,
      max: l,
      q1: s,
      median: u,
      q3: a,
      width: f,
      y: d,
      xAxis: h,
      yAxis: x,
    } = e,
    [g, v] = nt(n, h, x, { onlyOrthogonal: !0 }),
    j = ["top", "bottom"].includes(
      (t = n[h]) === null || t === void 0 ? void 0 : t.position
    );
  return {
    min: je(o, r, g),
    max: je(l, r, g),
    q1: je(s, r, g),
    median: je(u, r, g),
    q3: je(a, r, g),
    y: je(d, i, v),
    width: Pi(f, i, v),
    horizontal: j,
  };
}
function B1(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = ue(),
    { x1: i, y1: o, x2: l, y2: s, width: u, xAxis: a, yAxis: f } = e,
    [d, h] = nt(t, a, f, { onlyOrthogonal: !0 }),
    {
      x1: x,
      y1: g,
      x2: v,
      y2: j,
    } = { x1: je(i, n, d), x2: je(l, n, d), y1: je(o, n, h), y2: je(s, n, h) },
    { cx: m, cy: p } = { cx: (x + v) / 2, cy: (g + j) / 2 },
    y =
      (g > j ? -1 : 1) *
      (x > v ? -1 : 1) *
      Math.asin(Br([x, g], [x, p]) / Br([x, g], [m, p])),
    { widthX: w, widthY: S } = {
      widthX: (Math.sin(y) * Pi(u, r, d)) / 2,
      widthY: (Math.cos(y) * Pi(u, r, h)) / 2,
    };
  return {
    cx: m,
    cy: p,
    rx: Br([x, g], [v, j]) / 2,
    ry: Br([0, 0], [w, S]),
    rotation: Y1(y),
  };
}
function Y1(e) {
  return (e * 180) / Math.PI;
}
function Mn(e, t) {
  return e.endsWith("%") ? (Number(e.slice(0, -1)) * t) / 100 : Number(e);
}
function je(e, t, n) {
  return n === void 0 ? 0 : typeof e == "number" ? n(e) : Mn(e, t);
}
function Ff(e, t, n, r) {
  return r === void 0
    ? 0
    : Math.min(
        typeof t == "number" ? r(t) : Mn(t, n),
        typeof e == "number" ? r(e) : Mn(e, n)
      );
}
function Pi(e, t, n) {
  return n === void 0
    ? 0
    : Math.abs(typeof e == "number" ? n(0) - n(e) : Mn(e, t));
}
function If(e, t, n) {
  return n === void 0 ? 0 : typeof e == "number" ? n(e) - n(0) : Mn(e, t);
}
function Uf(e, t, n, r) {
  return r === void 0
    ? 0
    : Math.abs(
        (typeof t == "number" ? r(t) : Mn(t, n)) -
          (typeof e == "number" ? r(e) : Mn(e, n))
      );
}
function vl(e) {
  const [t] = Sr(),
    n = t.labels.find((r) => r.id === e);
  return n ? n.isVisible : !0;
}
function wl(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = ue(),
    { xAxis: i, yAxis: o, xShift: l, yShift: s } = e,
    [u, a] = nt(t, i, o, { onlyOrthogonal: !0 });
  return { xShift: If(l, n, u), yShift: If(s, r, a) };
}
function Q1({ color: e, id: t, width: n = 6 }) {
  return c.jsxs("defs", {
    children: [
      c.jsx("marker", {
        id: `marker-triangle-${t}`,
        viewBox: "0 0 10 10",
        refX: "5",
        refY: "5",
        markerWidth: n,
        markerHeight: n,
        orient: "auto-start-reverse",
        children: c.jsx("path", { fill: e, d: "M 0 0 L 10 5 L 0 10 z" }),
      }),
      c.jsx("marker", {
        id: `marker-circle-${t}`,
        viewBox: "0 0 10 10",
        refX: "5",
        refY: "5",
        markerWidth: n,
        markerHeight: n,
        children: c.jsx("circle", { cx: "5", cy: "5", r: "5", fill: e }),
      }),
      c.jsx("marker", {
        id: `marker-line-${t}`,
        viewBox: "0 0 10 10",
        refX: "5",
        refY: "5",
        markerWidth: n,
        markerHeight: n,
        orient: "auto",
        children: c.jsx("line", {
          x1: "5",
          x2: "5",
          y1: "0",
          y2: "10",
          stroke: e,
        }),
      }),
    ],
  });
}
function G1(e) {
  const {
      x1: t,
      y1: n,
      x2: r,
      y2: i,
      startPoint: o = "none",
      endPoint: l = "triangle",
      color: s = "black",
      strokeWidth: u,
      markerSize: a,
      xAxis: f = "x",
      yAxis: d = "y",
      ...h
    } = e,
    { x, y: g } = Cn({ x: t, y: n, xAxis: f, yAxis: d }),
    { x: v, y: j } = Cn({ x: r, y: i, xAxis: f, yAxis: d }),
    m = o !== "none" ? `url(#marker-${o}-${x}-${g}-${v}-${j})` : void 0,
    p = l !== "none" ? `url(#marker-${l}-${x}-${g}-${v}-${j})` : void 0;
  return c.jsxs("g", {
    children: [
      c.jsx(Q1, { color: s, id: `${x}-${g}-${v}-${j}`, width: a }),
      c.jsx("line", {
        x1: x,
        y1: g,
        x2: v,
        y2: j,
        stroke: s,
        strokeWidth: u,
        markerStart: m,
        markerEnd: p,
        ...h,
      }),
    ],
  });
}
function b1(e) {
  const {
      medianColor: t = "black",
      medianStyle: n,
      boxColor: r = "black",
      boxStyle: i,
      whiskerColor: o = "black",
      whiskerStyle: l,
      minMaxColor: s = "black",
      minMaxStyle: u,
      onMouseEnter: a,
      onMouseLeave: f,
      xAxis: d = "x",
      yAxis: h = "y",
      ...x
    } = e,
    {
      min: g,
      max: v,
      q1: j,
      median: m,
      q3: p,
      width: y,
      y: w,
      horizontal: S,
    } = V1({ ...x, xAxis: d, yAxis: h }),
    k = w - y / 2,
    E = w + y / 2,
    P = Math.abs(p - j);
  return c.jsxs("g", {
    onMouseEnter: a,
    onMouseLeave: f,
    children: [
      c.jsx("line", {
        x1: S ? g : w,
        x2: S ? j : w,
        y1: S ? w : g,
        y2: S ? w : j,
        stroke: o,
        style: l,
      }),
      c.jsx("line", {
        x1: S ? p : w,
        x2: S ? v : w,
        y1: S ? w : p,
        y2: S ? w : v,
        stroke: o,
        style: l,
      }),
      c.jsx("rect", {
        x: S ? Math.min(j, p) : k,
        y: S ? k : Math.min(j, p),
        width: S ? P : y,
        height: S ? y : P,
        stroke: r,
        style: i,
        fill: "none",
      }),
      c.jsx("line", {
        x1: S ? m : k,
        x2: S ? m : E,
        y1: S ? k : m,
        y2: S ? E : m,
        stroke: t,
        style: n,
      }),
      c.jsx("rect", {
        x: S ? Math.min(j, p) : k,
        y: S ? k : Math.min(j, p),
        width: S ? P : y,
        height: S ? y : P,
        stroke: r,
        style: { ...i, fill: "none" },
        fill: "none",
      }),
      c.jsx("line", {
        x1: S ? g : k,
        x2: S ? g : E,
        y1: S ? k : g,
        y2: S ? E : g,
        stroke: s,
        style: u,
      }),
      c.jsx("line", {
        x1: S ? v : k,
        x2: S ? v : E,
        y1: S ? k : v,
        y2: S ? E : v,
        stroke: s,
        style: u,
      }),
    ],
  });
}
function X1(e) {
  const { x: t, y: n, r, color: i, xAxis: o = "x", yAxis: l = "y", ...s } = e,
    {
      cx: u,
      cy: a,
      rx: f,
    } = $p({ cx: t, cy: n, rx: r, ry: r, xAxis: o, yAxis: l });
  return c.jsx("circle", { cx: u, cy: a, r: f, fill: i, ...s });
}
function K1(e) {
  const {
      x1: t,
      y1: n,
      y2: r,
      x2: i,
      color: o,
      width: l,
      style: s,
      xAxis: u = "x",
      yAxis: a = "y",
      ...f
    } = e,
    {
      cx: d,
      cy: h,
      rx: x,
      ry: g,
      rotation: v,
    } = B1({ x1: t, y1: n, y2: r, x2: i, width: l, xAxis: u, yAxis: a });
  return c.jsx("ellipse", {
    cx: d,
    cy: h,
    rx: x,
    ry: g,
    transform: `rotate(${v} 0 0)`,
    style: { ...s, transformOrigin: "center", transformBox: "fill-box" },
    fill: o,
    ...f,
  });
}
function Z1(e) {
  const {
      x: t,
      y: n,
      rx: r,
      ry: i,
      color: o,
      xAxis: l = "x",
      yAxis: s = "y",
      ...u
    } = e,
    {
      cx: a,
      cy: f,
      rx: d,
      ry: h,
    } = $p({ cx: t, cy: n, rx: r, ry: i, xAxis: l, yAxis: s });
  return c.jsx("ellipse", { cx: a, cy: f, rx: d, ry: h, fill: o, ...u });
}
function J1(e) {
  const {
      x: t,
      y: n,
      horizontalAlign: r = "none",
      verticalAlign: i = "none",
      style: o = {},
      xAxis: l = "x",
      yAxis: s = "y",
      children: u,
    } = e,
    { x: a, y: f } = Cn({ x: t, y: n, xAxis: l, yAxis: s });
  return c.jsx(Ai, {
    x: a,
    y: f,
    horizontalAlign: r,
    verticalAlign: i,
    style: o,
    children: u,
  });
}
function q1(e) {
  const {
      x1: t,
      x2: n,
      y1: r,
      y2: i,
      color: o = "black",
      xAxis: l = "x",
      yAxis: s = "y",
      ...u
    } = e,
    { x: a, y: f } = Cn({ x: t, y: r, xAxis: l, yAxis: s }),
    { x: d, y: h } = Cn({ x: n, y: i, xAxis: l, yAxis: s });
  return c.jsx("line", { x1: a, x2: d, y1: f, y2: h, stroke: o, ...u });
}
function ev(e) {
  const { points: t, color: n, xAxis: r = "x", yAxis: i = "y", ...o } = e,
    l = Np({ points: t, xAxis: r, yAxis: i });
  return c.jsx("polygon", { fill: n, points: l, ...o });
}
function tv(e) {
  const { points: t, color: n, xAxis: r = "x", yAxis: i = "y", ...o } = e,
    l = Np({ points: t, xAxis: r, yAxis: i });
  return c.jsx("polyline", { stroke: n, fill: "none", points: l, ...o });
}
function nv(e) {
  const {
      x1: t,
      y1: n,
      x2: r,
      y2: i,
      color: o,
      xAxis: l = "x",
      yAxis: s = "y",
      ...u
    } = e,
    {
      x: a,
      y: f,
      width: d,
      height: h,
    } = W1({ x1: t, y1: n, x2: r, y2: i, xAxis: l, yAxis: s });
  return c.jsx("rect", { x: a, y: f, width: d, height: h, fill: o, ...u });
}
const Ap = {
    circle: Dp,
    square: zp,
    diamond: Op,
    triangle: Rp,
    cross: Fp,
    xmark: ov,
    pentagon: lv,
    star: sv,
    hexagon: uv,
  },
  rv = Math.PI / 2,
  iv = Math.PI * 2;
function Dp({ style: e, size: t }) {
  return c.jsx("circle", { r: t / 2, style: e });
}
function zp({ style: e, size: t }) {
  const n = t / 2;
  return c.jsx("rect", { x: -n, y: -n, width: t, height: t, style: e });
}
function Op({ size: e, style: t }) {
  const n = e / 2;
  return c.jsx("polygon", {
    points: `0,${-n} ${n},0 0,${n} ${-n},0`,
    style: t,
  });
}
function Rp({ style: e, size: t }) {
  const n = (Math.sqrt(3) * t) / 2,
    r = t / 2;
  return c.jsx("polygon", {
    transform: `translate(0, -${(t - n) / 2})`,
    points: `${-r},${r} ${r},${r} 0,${r - n}`,
    style: e,
  });
}
function Fp({ size: e, style: t }) {
  const n = e / 2;
  return c.jsxs("g", {
    strokeWidth: 1,
    style: t,
    children: [
      c.jsx("line", { x1: 0, x2: 0, y1: n, y2: -n }),
      c.jsx("line", { x1: -n, x2: n, y1: 0, y2: 0 }),
    ],
  });
}
function ov(e) {
  return c.jsx("g", { transform: "rotate(45)", children: c.jsx(Fp, { ...e }) });
}
function lv({ style: e, size: t }) {
  return c.jsx("polygon", { points: Array.from(qo(t, 5)).join(" "), style: e });
}
function sv({ style: e, size: t }) {
  const n = Array.from(qo(t, 5, 0)),
    r = Array.from(qo(t / 2.5, 5, (2 * Math.PI) / 10)),
    i = [];
  for (let o = 0; o < n.length; o++) i.push(n[o], r[o]);
  return c.jsx("polygon", { points: i.join(" "), style: e });
}
function uv({ style: e, size: t }) {
  return c.jsx("polygon", { points: Array.from(qo(t, 6)).join(" "), style: e });
}
function* qo(e, t, n = 0) {
  const r = e / 2;
  for (let i = 0; i < t; i++) {
    const o = (iv * i) / t - rv + n,
      l = r * Math.cos(o),
      s = r * Math.sin(o);
    yield `${l},${s}`;
  }
}
const av = { triangle: cv, circle: fv, diamond: dv, square: hv };
function cv(e) {
  return c.jsx(Rp, { size: e.size, style: e.style });
}
function fv(e) {
  return c.jsx(Dp, { size: e.size, style: e.style });
}
function dv(e) {
  return c.jsx(Op, { size: e.size, style: e.style });
}
function hv(e) {
  return c.jsx(zp, { size: e.size, style: e.style });
}
function pv(e) {
  const {
      shape: t,
      x: n,
      y: r,
      onMouseEnter: i,
      onMouseLeave: o,
      color: l,
      size: s,
      style: u,
      xAxis: a = "x",
      yAxis: f = "y",
    } = e,
    d = av[t];
  if (!d) throw new Error(`Invalid shape: "${t}"`);
  const { x: h, y: x } = Cn({ x: n, y: r, xAxis: a, yAxis: f });
  return c.jsx("g", {
    onMouseEnter: i,
    onMouseLeave: o,
    transform: `translate(${h}, ${x})`,
    children: c.jsx(d, { size: s, style: { fill: l, ...u } }),
  });
}
function mv(e) {
  const {
      x: t,
      y: n,
      children: r,
      color: i,
      xAxis: o = "x",
      yAxis: l = "y",
      ...s
    } = e,
    { x: u, y: a } = Cn({ x: t, y: n, xAxis: o, yAxis: l });
  return c.jsx("text", { x: u, y: a, fill: i, ...s, children: r });
}
const Hf = {
  Arrow: G1,
  Circle: X1,
  DirectedEllipse: K1,
  Ellipse: Z1,
  Group: J1,
  Line: q1,
  Rectangle: nv,
  Shape: pv,
  Text: mv,
  Polyline: tv,
  Polygon: ev,
  BoxPlot: b1,
};
function Ip(e) {
  return c.jsx(c.Fragment, { children: e.children });
}
function Up(e, t, n, r) {
  return e || t ? 0 : yv(n, r);
}
function yv(e, t) {
  switch (e) {
    case "outer":
      return 0;
    case "inner":
      return t;
    case "center":
      return t / 2;
    default:
      throw new Error("unreachable");
  }
}
const Jn = _.createContext(null);
function gv(e) {
  const {
      plotHeight: t,
      style: n,
      primaryTicks: r,
      position: i,
      primaryGrid: o,
      secondaryGrid: l,
      secondaryStyle: s,
      scale: u,
    } = e,
    a = _.useMemo(() => {
      const f = [];
      if (o)
        for (const { position: d } of r)
          f.push(
            c.jsx(
              "line",
              {
                x1: d,
                x2: d,
                y1: i === "top" ? t : -t,
                y2: "0",
                stroke: "black",
                strokeDasharray: "2,2",
                strokeOpacity: 0.5,
                style: n,
              },
              d
            )
          );
      if (l) {
        const h = (u == null ? void 0 : u.ticks(r.length * 5)) || [];
        for (const x of h) {
          const g = u == null ? void 0 : u(x);
          if (!g || r.map((v) => v.position).includes(g)) return null;
          f.push(
            c.jsx(
              "line",
              {
                x1: g,
                x2: g,
                y1: i === "top" ? t : -t,
                y2: "0",
                stroke: "black",
                strokeDasharray: "5",
                strokeOpacity: 0.2,
                style: s,
              },
              g
            )
          );
        }
      }
      return f;
    }, [i, t, o, r, u, l, s, n]);
  return c.jsx("g", { children: a });
}
function xv(e) {
  const { plotWidth: t, label: n, labelStyle: r, verticalAlign: i } = e;
  return c.jsx(Ai, {
    x: t / 2,
    y: 0,
    horizontalAlign: "middle",
    verticalAlign: i,
    children: c.jsx("text", { textAnchor: "middle", style: r, children: n }),
  });
}
function vv(e) {
  const { plotWidth: t, style: n } = e;
  return c.jsx("line", { x1: 0, x2: t, stroke: "black", style: n });
}
function Hp(e) {
  const {
      primaryTicks: t,
      getPositions: n,
      secondaryTickLength: r,
      scale: i,
      secondaryTickStyle: o,
      style: l,
      ...s
    } = e,
    u = t.map((f) => {
      const { line: d, text: h } = n(f.position);
      return c.jsx(
        Wf,
        { line: d, style: l, text: h, ...s, children: f.label },
        f.position
      );
    });
  let a = [];
  if (r !== 0) {
    const h =
      (Math.abs(
        (i == null ? void 0 : i.range()[1]) -
          (i == null ? void 0 : i.range()[0])
      ) || 0) /
        t.length <
      50
        ? 5
        : 10;
    a =
      ((i == null ? void 0 : i.ticks(t.length * h)) || []).map((g) => {
        if (t.map((m) => m.position).includes(i(g))) return null;
        const { line: v, text: j } = n(i(g), !0);
        return c.jsx(
          Wf,
          { line: v, text: j, secondary: !0, style: o, ...s },
          String(g)
        );
      }) || [];
  }
  return c.jsxs(c.Fragment, { children: [a, u] });
}
function Wf(e) {
  const {
    line: { x1: t = 0, x2: n = 0, y1: r = 0, y2: i = 0 },
    text: { x1: o = 0, y1: l = 0 },
    children: s,
    strokeColor: u = "black",
    strokeHeight: a = 1,
    anchor: f = "end",
    secondary: d = !1,
    labelStyle: h,
    style: x,
    dominantBaseline: g = "middle",
  } = e;
  return c.jsxs("g", {
    children: [
      c.jsx("line", {
        x1: t,
        x2: d && t !== n ? n * a : n,
        y1: r,
        y2: d && r !== i ? i * a : i,
        stroke: u,
        strokeWidth: d ? 1 : 1.5,
        style: x,
      }),
      !d &&
        c.jsx("text", {
          x: o,
          y: l,
          textAnchor: f,
          dominantBaseline: g,
          style: h,
          children: s,
        }),
    ],
  });
}
function $a(e) {
  const {
      primaryTickStyle: t,
      primaryTickLength: n,
      tickPosition: r,
      hiddenTicks: i,
      lineStyle: o,
      hiddenLine: l,
      hidden: s,
      plotWidth: u,
      plotHeight: a,
      displayPrimaryGridLines: f,
      displaySecondaryGridLines: d,
      primaryGridLineStyle: h,
      secondaryGridLineStyle: x,
      label: g,
      labelStyle: v,
      axisRef: j,
      primaryTicks: m,
      position: p,
      tickLabelStyle: y,
      innerOffset: w,
      secondaryTickLength: S,
      scale: k,
      secondaryTickStyle: E,
    } = e,
    P = p === "bottom",
    D = 0,
    O = P ? `translate(0, ${a})` : void 0,
    N = pt();
  function W(L, z = !1) {
    const { y1: F, y2: X, textPosition: _e } = R(z);
    return {
      line: { x1: L, x2: L, y1: F, y2: X },
      text: { x1: L, y1: P ? _e : -_e },
    };
  }
  function R(L = !1) {
    const z = L ? S : n,
      F = P ? z : -z;
    switch (r) {
      case "center":
        return { y1: F / 2, y2: -F / 2, textPosition: D + z / 2 };
      case "inner":
        return { y1: 0, y2: -F, textPosition: D };
      case "outer":
        return { y1: 0, y2: F, textPosition: D + z };
      default:
        return { y1: 0, y: 0, textPosition: 0 };
    }
  }
  const Q =
      f || d
        ? c.jsx(gv, {
            position: p,
            plotHeight: a,
            primaryTicks: m,
            style: h,
            primaryGrid: f,
            secondaryGrid: d,
            secondaryStyle: x,
            scale: k,
          })
        : null,
    K =
      !s && !i
        ? c.jsx(Hp, {
            anchor: "middle",
            primaryTicks: m,
            getPositions: W,
            labelStyle: y,
            style: t,
            secondaryTickLength: S,
            scale: k,
            secondaryTickStyle: E,
            dominantBaseline: P ? "text-before-edge" : "text-after-edge",
          })
        : null,
    b = !s && !l ? c.jsx(vv, { style: o, plotWidth: u }) : null,
    le =
      !s && g
        ? c.jsx(xv, {
            plotWidth: u,
            label: g,
            labelStyle: v,
            verticalAlign: P ? "start" : "end",
          })
        : null,
    T = _.useContext(Jn);
  return c.jsxs("g", {
    transform: O,
    children: [
      Q,
      c.jsxs("g", {
        ref: T,
        children: [
          c.jsx("g", { ref: N.ref, children: K }),
          c.jsx("g", { ref: j, children: b }),
          c.jsx("g", {
            transform: `translate(0, ${(N.height - w) * (P ? 1 : -1)})`,
            children: le,
          }),
        ],
      }),
    ],
  });
}
function wv(e) {
  const {
      plotWidth: t,
      style: n,
      primaryTicks: r,
      position: i,
      primaryGrid: o,
      secondaryGrid: l,
      scale: s,
      secondaryStyle: u,
    } = e,
    a = _.useMemo(() => {
      const f = [];
      if (o)
        for (const { position: d } of r)
          f.push(
            c.jsx(
              "line",
              {
                x1: "0",
                x2: i === "left" ? t : -t,
                y1: d,
                y2: d,
                stroke: "black",
                strokeDasharray: "2,2",
                strokeOpacity: 0.5,
                style: n,
              },
              d
            )
          );
      if (l) {
        const h = (s == null ? void 0 : s.ticks(r.length * 5)) || [];
        for (const x of h) {
          const g = (s == null ? void 0 : s(x)) || 0;
          if (r.map((v) => v.position).includes(g)) return null;
          f.push(
            c.jsx(
              "line",
              {
                x1: "0",
                x2: i === "left" ? t : -t,
                y1: g,
                y2: g,
                stroke: "black",
                strokeDasharray: "5",
                strokeOpacity: 0.2,
                style: u,
              },
              g
            )
          );
        }
      }
      return f;
    }, [i, t, o, r, s, l, u, n]);
  return c.jsx("g", { children: a });
}
function Sv(e) {
  const { transform: t = "", label: n, ...r } = e;
  return c.jsx("text", {
    ...r,
    transform: `${t}rotate(-90)`,
    textAnchor: "middle",
    children: n,
  });
}
function kv(e) {
  const { plotHeight: t, label: n, labelStyle: r, horizontalAlign: i } = e;
  return c.jsx(Ai, {
    x: 0,
    y: t / 2,
    horizontalAlign: i,
    verticalAlign: "middle",
    children: c.jsx(Sv, { label: n, style: r }),
  });
}
function jv(e) {
  const { plotHeight: t, style: n } = e;
  return c.jsx("line", { y1: 0, y2: t, stroke: "black", style: n });
}
function Aa(e) {
  const {
      primaryTickStyle: t,
      primaryTickLength: n,
      tickPosition: r,
      hiddenTicks: i,
      primaryGridLineStyle: o,
      lineStyle: l,
      hiddenLine: s,
      hidden: u,
      plotWidth: a,
      plotHeight: f,
      displayPrimaryGridLines: d,
      displaySecondaryGridLines: h,
      secondaryGridLineStyle: x,
      label: g,
      labelStyle: v,
      axisRef: j,
      primaryTicks: m,
      position: p,
      tickLabelStyle: y,
      innerOffset: w,
      secondaryTickLength: S,
      scale: k,
      secondaryTickStyle: E,
    } = e,
    P = p === "right",
    D = 3,
    O = P ? `translate(${a}, 0)` : void 0,
    N = pt();
  function W(L, z = !1) {
    const { x1: F, x2: X, textPosition: _e } = R(z);
    return {
      line: { y1: L, y2: L, x1: F, x2: X },
      text: { y1: L, x1: P ? _e : -_e },
    };
  }
  function R(L = !1) {
    const z = L ? S : n,
      F = P ? z : -z;
    switch (r) {
      case "center":
        return { x1: F / 2, x2: -F / 2, textPosition: D + z / 2 };
      case "inner":
        return { x1: 0, x2: -F, textPosition: D };
      case "outer":
        return { x1: 0, x2: F, textPosition: D + z };
      default:
        return { x1: 0, x2: 0, textPosition: 3 };
    }
  }
  const Q =
      d || h
        ? c.jsx(wv, {
            position: p,
            plotWidth: a,
            primaryTicks: m,
            style: o,
            primaryGrid: d,
            secondaryGrid: h,
            secondaryStyle: x,
            scale: k,
          })
        : null,
    K =
      !u && !i
        ? c.jsx(Hp, {
            anchor: P ? "start" : "end",
            primaryTicks: m,
            getPositions: W,
            labelStyle: y,
            style: t,
            secondaryTickLength: S,
            scale: k,
            secondaryTickStyle: E,
          })
        : null,
    b = !u && !s ? c.jsx(jv, { style: l, plotHeight: f }) : null,
    le =
      !u && g
        ? c.jsx(kv, {
            plotHeight: f,
            label: g,
            labelStyle: v,
            horizontalAlign: P ? "start" : "end",
          })
        : null,
    T = _.useContext(Jn);
  return c.jsxs("g", {
    transform: O,
    children: [
      Q,
      c.jsxs("g", {
        ref: T,
        children: [
          c.jsx("g", { ref: N.ref, children: K }),
          c.jsx("g", { ref: j, children: b }),
          c.jsx("g", {
            transform: `translate(${(N.width - w) * (P ? 1 : -1)}, 0)`,
            children: le,
          }),
        ],
      }),
    ],
  });
}
function Ev(e) {
  const { position: t, tickLabelFormat: n, scale: r, ...i } = e,
    o = _.useRef(null),
    l = t === "left" || t === "right" ? "vertical" : "horizontal",
    s = w1(r, l, o, { tickFormat: n }),
    u = l === "vertical" ? Aa : $a;
  return c.jsx(u, { scale: r, axisRef: o, primaryTicks: s, position: t, ...i });
}
const Wp = _.memo(Ev);
function Pv(e) {
  const { position: t, tickLabelFormat: n, scale: r, ...i } = e,
    o = _.useRef(null),
    l = t === "left" || t === "right" ? "vertical" : "horizontal",
    s = j1(r, l, o, { tickFormat: n }),
    u = l === "vertical" ? Aa : $a;
  return c.jsx(u, { scale: r, axisRef: o, primaryTicks: s, position: t, ...i });
}
const Vp = _.memo(Pv);
function Cv(e) {
  const { position: t, tickLabelFormat: n, scale: r, ...i } = e,
    o = _.useRef(null),
    l = t === "left" || t === "right" ? "vertical" : "horizontal",
    s = E1(r, l, o, { tickFormat: n }),
    u = l === "vertical" ? Aa : $a;
  return c.jsx(u, { scale: r, axisRef: o, primaryTicks: s, position: t, ...i });
}
const Bp = _.memo(Cv);
function xn({
  id: e,
  position: t,
  min: n,
  max: r,
  paddingStart: i = 0,
  paddingEnd: o = 0,
  flip: l = !1,
  scale: s = "linear",
  label: u,
  displayPrimaryGridLines: a = !1,
  primaryGridLineStyle: f,
  displaySecondaryGridLines: d,
  secondaryGridLineStyle: h,
  labelStyle: x,
  hidden: g = !1,
  tickLabelStyle: v,
  tickLabelFormat: j = s === "time" ? void 0 : String,
  hiddenLine: m = !1,
  lineStyle: p,
  hiddenTicks: y = !1,
  tickPosition: w = "outer",
  primaryTickLength: S = 5,
  primaryTickStyle: k,
  secondaryTickLength: E = 3,
  secondaryTickStyle: P,
}) {
  const D = Di(),
    { axisContext: O, plotWidth: N, plotHeight: W } = ue(),
    R = ["top", "bottom"].includes(t) ? "x" : "y",
    Q = e || R,
    K = Up(g, y, w, S);
  _.useEffect(
    () => (
      D({
        type: "addAxis",
        payload: {
          id: Q,
          position: t,
          min: n,
          max: r,
          paddingStart: i,
          paddingEnd: o,
          flip: l,
          scale: s,
          innerOffset: K,
          tickLabelFormat: j,
        },
      }),
      () => D({ type: "removeAxis", payload: { id: Q } })
    ),
    [D, l, K, r, n, o, i, t, s, Q, j]
  );
  const b = O[Q];
  if (!b) return null;
  const le = {
    hidden: g,
    plotWidth: N,
    plotHeight: W,
    displayPrimaryGridLines: a,
    label: u,
    labelStyle: x,
    tickLabelStyle: v,
    position: t,
    hiddenLine: m,
    primaryGridLineStyle: f,
    displaySecondaryGridLines: d,
    secondaryGridLineStyle: h,
    lineStyle: p,
    hiddenTicks: y,
    tickPosition: w,
    primaryTickLength: S,
    primaryTickStyle: k,
    innerOffset: K,
    secondaryTickLength: E,
    secondaryTickStyle: P,
  };
  return s === "linear"
    ? c.jsx(Wp, { ...le, tickLabelFormat: j, scale: b.scale })
    : s === "time"
    ? c.jsx(Bp, { ...le, tickLabelFormat: j, scale: b.scale })
    : c.jsx(Vp, { ...le, tickLabelFormat: j, scale: b.scale });
}
function Mv(e) {
  switch (e) {
    case "bottom":
      return "top";
    case "top":
      return "bottom";
    case "left":
      return "right";
    case "right":
      return "left";
    default:
      throw new Error(`Unknown position ${e}`);
  }
}
function Tv(e) {
  const {
      id: t,
      hidden: n = !1,
      primaryTickLength: r = 5,
      secondaryTickLength: i = 3,
      tickPosition: o = "outer",
      hiddenLine: l = !1,
      hiddenTicks: s = !1,
      tickLabelFormat: u,
      ...a
    } = e,
    { axisContext: f, plotWidth: d, plotHeight: h } = ue(),
    x = Up(n, s, o, r),
    g = f[t];
  if (!g) return null;
  const v = Mv(g.position),
    { type: j, scale: m, tickLabelFormat: p } = g,
    y = u ?? p,
    w = {
      plotWidth: d,
      plotHeight: h,
      position: v,
      displayPrimaryGridLines: !1,
      hidden: n,
      primaryTickLength: r,
      tickPosition: o,
      hiddenLine: l,
      hiddenTicks: s,
      innerOffset: x,
      secondaryTickLength: i,
      ...a,
    };
  return j === "linear"
    ? c.jsx(Wp, { ...w, tickLabelFormat: y, scale: m })
    : j === "time"
    ? c.jsx(Bp, { ...w, tickLabelFormat: y, scale: m })
    : c.jsx(Vp, { ...w, tickLabelFormat: y, scale: m });
}
function Sl({
  title: e,
  titleStyle: t,
  titleClass: n,
  subtitle: r,
  subtitleStyle: i,
  subtitleClass: o,
  position: l = "top",
}) {
  const { width: s, height: u } = ue(),
    a = Di(),
    f = {
      dominantBaseline: "hanging",
      textAnchor: "middle",
      fontSize: "16px",
      fontWeight: "bold",
    },
    d = {
      dominantBaseline: "hanging",
      textAnchor: "middle",
      fontSize: "14px",
      color: "gray",
    },
    h = pt();
  return (
    _.useEffect(
      () => (
        a({ type: "addHeading", payload: { position: l } }),
        () => a({ type: "removeHeading" })
      ),
      [a, l]
    ),
    c.jsxs(Ai, {
      x: s / 2,
      y: l === "top" ? 0 : u,
      horizontalAlign: "middle",
      verticalAlign: l === "top" ? "start" : "end",
      children: [
        c.jsx("text", {
          ref: h.ref,
          style: { ...f, ...t },
          className: n,
          children: e,
        }),
        r &&
          c.jsx("text", {
            transform: `translate(0, ${h.height})`,
            style: { ...d, ...i },
            className: o,
            children: r,
          }),
      ],
    })
  );
}
Sl.defaultProps = { position: "top" };
const Yp = _.createContext(0);
function On(e, t, n, r) {
  if (e[t] !== void 0) {
    if (e[n] !== void 0)
      throw new Error(
        `${t} and ${n} should't be both defined for the position ${r}`
      );
    return { key: t, value: e[t] };
  }
  return e[n] !== void 0 ? { key: n, value: e[n] } : {};
}
function _v(e, t, n, r, i) {
  switch (e) {
    case "embedded": {
      const { key: o = "top", value: l = 10 } = On(t, "top", "bottom", e),
        { key: s = "left", value: u = 10 } = On(t, "left", "right", e),
        a = s === "right" ? n - u : u,
        f = o === "bottom" ? r - l : l;
      return {
        x: a,
        y: f,
        horizontalAlign: s === "left" ? "start" : "end",
        verticalAlign: o === "top" ? "start" : "end",
      };
    }
    case "top": {
      const { value: o = n / 2 } = On(t, "left", "right", e),
        l = o,
        s = -(t.bottom || 0) - i;
      return { x: l, y: s, horizontalAlign: "middle", verticalAlign: "end" };
    }
    case "right": {
      const { key: o = "top", value: l = r / 2 } = On(t, "top", "bottom", e),
        s = o === "bottom" ? -l : l;
      return {
        x: n + (t.left || 0) + i,
        y: s,
        horizontalAlign: "start",
        verticalAlign: "middle",
      };
    }
    case "bottom": {
      const { value: o = n / 2 } = On(t, "left", "right", e),
        l = o,
        s = r + (t.top || 0) + i;
      return { x: l, y: s, horizontalAlign: "middle", verticalAlign: "start" };
    }
    case "left": {
      const { key: o = "top", value: l = r / 2 } = On(t, "top", "bottom", e),
        s = o === "bottom" ? -l : l;
      return {
        x: -(t.right || 0) - i,
        y: s,
        horizontalAlign: "end",
        verticalAlign: "middle",
      };
    }
    default:
      throw new Error(`Position ${JSON.stringify(e)} unknown`);
  }
}
function Qp(e) {
  const {
      position: t = "embedded",
      margin: n = 10,
      onClick: r,
      lineStyle: i = {},
      showHide: o = !1,
      labelStyle: l = {},
      ...s
    } = e,
    { plotWidth: u, plotHeight: a } = ue(),
    f = Di(),
    [d, h] = Sr(),
    x = _.useContext(Yp),
    g = _.useMemo(() => _v(t, s, u, a, x), [t, s, u, a, x]);
  _.useEffect(
    () => (
      f({ type: "addLegend", payload: { position: t, margin: n } }),
      () => f({ type: "removeLegend" })
    ),
    [f, t, n]
  );
  function v(j, m) {
    r == null || r({ event: j, id: m }),
      o && h({ type: "TOGGLE_VISIBILITY", payload: { id: m } });
  }
  return c.jsx(Ai, {
    ...g,
    children: d.labels.map((j, m) => {
      const p = gn({ fontSize: "16px" }, l, { id: j.id }),
        y = gn({}, i, { id: j.id }),
        w = Number.parseInt(String(p.fontSize), 10),
        S = 10,
        k = m * w + w / 2 + 3;
      if (j.range)
        return c.jsxs(
          "g",
          {
            onClick: (P) => v(P, j.id),
            transform: `translate(${S}, 0)`,
            style: { opacity: j.isVisible ? "1" : "0.6" },
            children: [
              Nv({
                index: m,
                rangeColor: j.range.rangeColor,
                lineColor: j.colorLine,
                style: y,
                height: w,
              }),
              c.jsx("text", {
                style: p,
                x: 30,
                y: `${(m + 1) * w}`,
                children: j.label,
              }),
            ],
          },
          m
        );
      let E;
      return (
        j.shape && (E = Ap[j.shape.figure]),
        c.jsxs(
          "g",
          {
            onClick: (P) => v(P, j.id),
            transform: `translate(${S}, 0)`,
            style: { opacity: j.isVisible ? "1" : "0.6" },
            children: [
              Lv({ index: m, color: j.colorLine, style: y, height: w }),
              c.jsx("g", {
                transform: `translate(${S - 1}, ${k})`,
                children:
                  j.shape &&
                  E &&
                  !j.shape.hidden &&
                  c.jsx(E, { size: 10, style: { fill: j.shape.color } }),
              }),
              c.jsx("text", {
                style: p,
                x: 30,
                y: `${(m + 1) * w}`,
                children: j.label,
              }),
            ],
          },
          m
        )
      );
    }),
  });
}
function Lv(e) {
  const { index: t, color: n, style: r = {}, height: i = 16 } = e,
    o = 0,
    { strokeWidth: l = "2px" } = r,
    s = t * i + i / 2 + 3;
  return c.jsx("line", {
    x1: o,
    x2: o + 20,
    y1: s,
    y2: s,
    stroke: n,
    style: { ...r, strokeWidth: l },
  });
}
function Nv(e) {
  const {
      index: t,
      rangeColor: n,
      lineColor: r,
      style: i = {},
      height: o = 16,
    } = e,
    { strokeWidth: l = "15px" } = i,
    s = Number.parseInt(l == null ? void 0 : l.toString(), 10) / 5,
    u = 0,
    a = t * o + o / 2 - s;
  return c.jsxs("g", {
    transform: `translate(${u}, ${a})`,
    children: [
      c.jsx("rect", { width: "20", height: s * 3, fill: n }),
      c.jsx("line", {
        style: i,
        x1: 0,
        y: 0,
        x2: 20,
        y2: 0,
        stroke: r,
        strokeWidth: s,
      }),
      c.jsx("line", {
        style: i,
        x1: 0,
        y1: s * 3,
        x2: 20,
        y2: s * 3,
        stroke: r,
        strokeWidth: s,
      }),
    ],
  });
}
function $v(e) {
  const {
      xAxis: t = "x",
      yAxis: n = "y",
      data: r,
      hidden: i,
      transform: o,
      ...l
    } = e,
    { axisContext: s } = ue(),
    [u, a] = nt(s, t, n),
    f = _.useMemo(
      () =>
        i || u === void 0 || a === void 0
          ? null
          : r.map((h, x) => {
              const g = zf(h.xError),
                v = zf(h.yError);
              return c.jsx(
                Av,
                {
                  origin: { x: u(h.x), y: a(h.y) },
                  bottom: v ? a(h.y - (v[0] || 0)) : null,
                  top: v ? a(h.y + (v[1] || 0)) : null,
                  left: g ? u(h.x - (g[0] || 0)) : null,
                  right: g ? u(h.x + (g[1] || 0)) : null,
                  ...l,
                },
                `ErrorBars-${x}`
              );
            }),
      [r, u, a, i, l]
    );
  return c.jsx("g", { transform: o, children: f });
}
function Av(e) {
  const {
      origin: t,
      top: n,
      bottom: r,
      left: i,
      right: o,
      style: l,
      capSize: s = 10,
      capStyle: u,
    } = e,
    a = "black";
  return c.jsxs("g", {
    children: [
      n != null &&
        c.jsxs("g", {
          children: [
            c.jsx("line", {
              x1: t.x,
              x2: t.x,
              y1: t.y,
              y2: n,
              stroke: a,
              ...l,
            }),
            c.jsx("line", {
              x1: t.x - s / 2,
              x2: t.x + s / 2,
              y1: n,
              y2: n,
              stroke: a,
              ...l,
              ...u,
            }),
          ],
        }),
      r != null &&
        c.jsxs("g", {
          children: [
            c.jsx("line", {
              x1: t.x,
              x2: t.x,
              y1: t.y,
              y2: r,
              stroke: a,
              ...l,
            }),
            c.jsx("line", {
              x1: t.x - s / 2,
              x2: t.x + s / 2,
              y1: r,
              y2: r,
              stroke: a,
              ...l,
              ...u,
            }),
          ],
        }),
      i != null &&
        c.jsxs("g", {
          children: [
            c.jsx("line", {
              x1: t.x,
              x2: i,
              y1: t.y,
              y2: t.y,
              stroke: a,
              ...l,
            }),
            c.jsx("line", {
              x1: i,
              x2: i,
              y1: t.y - s / 2,
              y2: t.y + s / 2,
              stroke: a,
              ...l,
              ...u,
            }),
          ],
        }),
      o != null &&
        c.jsxs("g", {
          children: [
            c.jsx("line", {
              x1: t.x,
              x2: o,
              y1: t.y,
              y2: t.y,
              stroke: a,
              ...l,
            }),
            c.jsx("line", {
              x1: o,
              x2: o,
              y1: t.y - s / 2,
              y2: t.y + s / 2,
              stroke: a,
              ...l,
              ...u,
            }),
          ],
        }),
    ],
  });
}
function Da(e) {
  const t = Di(),
    { colorScaler: n } = ue(),
    [, r] = Sr(),
    i = kr(e.id, "series"),
    {
      xAxis: o = "x",
      yAxis: l = "y",
      data: s,
      label: u,
      hidden: a,
      displayErrorBars: f = !1,
      xShift: d = 0,
      yShift: h = 0,
      ...x
    } = e,
    {
      markerShape: g = "circle",
      markerStyle: v = {},
      errorBarsStyle: j,
      errorBarsCapStyle: m,
      errorBarsCapSize: p,
    } = x,
    y = vl(i),
    { xShift: w, yShift: S } = wl({ xAxis: o, yAxis: l, xShift: d, yShift: h }),
    k = `translate(${w},${S})`;
  if (
    (_.useEffect(() => {
      var D;
      if (!a)
        return (
          r({
            type: "ADD_LEGEND_LABEL",
            payload: {
              id: i,
              label: u,
              colorLine: "white",
              shape: {
                color:
                  ((D = v == null ? void 0 : v.fill) === null || D === void 0
                    ? void 0
                    : D.toString()) || n(i),
                figure: typeof g == "string" ? g : "circle",
                hidden: !1,
              },
            },
          }),
          () => r({ type: "REMOVE_LEGEND_LABEL", payload: { id: i } })
        );
    }, [n, a, i, u, r, g, v == null ? void 0 : v.fill]),
    _.useEffect(() => {
      const [D, O] = ti(s, (K) => K.x),
        [N, W] = ti(s, (K) => K.y);
      return (
        t({
          type: "addSeries",
          payload: {
            id: i,
            x: { min: D, max: O, axisId: o, shift: d },
            y: { min: N, max: W, axisId: l, shift: h },
            label: u,
            data: s,
          },
        }),
        () => t({ type: "removeSeries", payload: { id: i } })
      );
    }, [t, i, s, o, l, u, d, h]),
    a)
  )
    return null;
  const E = { data: s, xAxis: o, yAxis: l },
    P = { hidden: !f, style: j, capStyle: m, capSize: p, transform: k };
  return y
    ? c.jsxs("g", {
        children: [
          c.jsx($v, { ...E, ...P }),
          c.jsx(Dv, { ...x, ...E, id: i, transform: k }),
        ],
      })
    : null;
}
function Dv({
  id: e,
  data: t,
  xAxis: n,
  yAxis: r,
  markerShape: i = "circle",
  markerSize: o = 8,
  markerStyle: l = {},
  pointLabel: s = "",
  pointLabelStyle: u = {},
  displayMarkers: a = !0,
  lineStyle: f = {},
  displayLines: d = !1,
  transform: h,
}) {
  const { axisContext: x, colorScaler: g } = ue(),
    [v, j] = nt(x, n, r),
    m = _.useMemo(() => {
      if (v === void 0 || j === void 0) return null;
      const p = g(e),
        y = { fill: p, stroke: p };
      return t.map((S, k) => {
        const E = gn(y, l, S, k, t),
          P = Ap[N1(i, S, k, t)],
          D = $1(s, S, k, t),
          O = gn({}, u, S, k, t),
          N = [];
        if (d) {
          const W = gn({}, f, S, k, t),
            R = k > 0 ? Of(S, t[k - 1]) : void 0,
            Q = t[k + 1] ? Of(S, t[k + 1]) : void 0,
            K = R
              ? c.jsx(
                  "line",
                  {
                    x1: 0,
                    y1: 0,
                    x2: v(R.x) - v(S.x),
                    y2: j(R.y) - j(S.y),
                    style: { stroke: E.fill, ...W },
                  },
                  `markers-${k}-previous`
                )
              : null,
            b = Q
              ? c.jsx(
                  "line",
                  {
                    x1: 0,
                    y1: 0,
                    x2: v(Q.x) - v(S.x),
                    y2: j(Q.y) - j(S.y),
                    style: { stroke: E.fill, ...W },
                  },
                  `markers-${k}-next`
                )
              : null;
          N.push(K, b);
        }
        return c.jsxs(
          "g",
          {
            transform: `translate(${v(S.x)}, ${j(S.y)})`,
            children: [
              N,
              a ? c.jsx(P, { size: o, style: { stroke: E.fill, ...E } }) : null,
              D ? c.jsx("text", { style: O, children: D }) : null,
            ],
          },
          `markers-${k}`
        );
      });
    }, [v, j, g, e, t, l, i, s, u, f, d, a, o]);
  return m
    ? c.jsx("g", { transform: h, className: "markers", children: m })
    : null;
}
function zv(e) {
  var t;
  const [, n] = Sr(),
    { colorScaler: r } = ue(),
    i = kr(e.id, "series"),
    { lineStyle: o = {}, displayMarkers: l = !1, hidden: s, ...u } = e,
    {
      xAxis: a = "x",
      yAxis: f = "y",
      xShift: d = "0",
      yShift: h = "0",
      pointLabel: x,
      displayErrorBars: g,
      label: v,
      markerStyle: j,
      markerShape: m,
    } = u,
    { xShift: p, yShift: y } = wl({ xAxis: a, yAxis: f, xShift: d, yShift: h }),
    w = {
      id: i,
      data: e.data,
      xAxis: a,
      yAxis: f,
      lineStyle: gn({}, o, { id: i }),
      transform: `translate(${p},${y})`,
    },
    S =
      o != null && o.stroke ? (o == null ? void 0 : o.stroke.toString()) : r(i),
    k =
      ((t = j == null ? void 0 : j.fill) === null || t === void 0
        ? void 0
        : t.toString()) || r(i),
    E = m || "circle",
    P = _.useMemo(() => ({ color: k, figure: E, hidden: !l }), [k, l, E]),
    D = vl(i);
  return (
    _.useEffect(() => {
      if (!s)
        return (
          n({
            type: "ADD_LEGEND_LABEL",
            payload: { id: i, label: v, colorLine: S, shape: P },
          }),
          () => n({ type: "REMOVE_LEGEND_LABEL", payload: { id: i } })
        );
    }, [S, n, v, P, i, s]),
    s
      ? null
      : c.jsxs("g", {
          children: [
            D && c.jsx(Ov, { ...w }),
            c.jsx(Da, {
              ...u,
              hidden: !l && !x && !g,
              displayMarkers: l,
              id: i,
            }),
          ],
        })
  );
}
function Ov({
  id: e,
  data: t,
  xAxis: n,
  yAxis: r,
  lineStyle: i,
  transform: o,
}) {
  const { axisContext: l, colorScaler: s } = ue(),
    [u, a] = nt(l, n, r),
    f = s(e);
  if (u === void 0 || a === void 0) return null;
  const d = { stroke: f, strokeWidth: 2, ...i };
  return c.jsx("g", {
    transform: o,
    children: t.map(({ x: h, y: x }) =>
      c.jsx(
        "line",
        { style: d, x1: u(h), x2: u(h), y1: a(x), y2: a(0) },
        `${h}-${x}`
      )
    ),
  });
}
function he(e) {
  return function () {
    return e;
  };
}
const mu = Math.PI,
  yu = 2 * mu,
  un = 1e-6,
  Rv = yu - un;
function Gp(e) {
  this._ += e[0];
  for (let t = 1, n = e.length; t < n; ++t) this._ += arguments[t] + e[t];
}
function Fv(e) {
  let t = Math.floor(e);
  if (!(t >= 0)) throw new Error(`invalid digits: ${e}`);
  if (t > 15) return Gp;
  const n = 10 ** t;
  return function (r) {
    this._ += r[0];
    for (let i = 1, o = r.length; i < o; ++i)
      this._ += Math.round(arguments[i] * n) / n + r[i];
  };
}
class Iv {
  constructor(t) {
    (this._x0 = this._y0 = this._x1 = this._y1 = null),
      (this._ = ""),
      (this._append = t == null ? Gp : Fv(t));
  }
  moveTo(t, n) {
    this._append`M${(this._x0 = this._x1 = +t)},${(this._y0 = this._y1 = +n)}`;
  }
  closePath() {
    this._x1 !== null &&
      ((this._x1 = this._x0), (this._y1 = this._y0), this._append`Z`);
  }
  lineTo(t, n) {
    this._append`L${(this._x1 = +t)},${(this._y1 = +n)}`;
  }
  quadraticCurveTo(t, n, r, i) {
    this._append`Q${+t},${+n},${(this._x1 = +r)},${(this._y1 = +i)}`;
  }
  bezierCurveTo(t, n, r, i, o, l) {
    this._append`C${+t},${+n},${+r},${+i},${(this._x1 = +o)},${(this._y1 =
      +l)}`;
  }
  arcTo(t, n, r, i, o) {
    if (((t = +t), (n = +n), (r = +r), (i = +i), (o = +o), o < 0))
      throw new Error(`negative radius: ${o}`);
    let l = this._x1,
      s = this._y1,
      u = r - t,
      a = i - n,
      f = l - t,
      d = s - n,
      h = f * f + d * d;
    if (this._x1 === null) this._append`M${(this._x1 = t)},${(this._y1 = n)}`;
    else if (h > un)
      if (!(Math.abs(d * u - a * f) > un) || !o)
        this._append`L${(this._x1 = t)},${(this._y1 = n)}`;
      else {
        let x = r - l,
          g = i - s,
          v = u * u + a * a,
          j = x * x + g * g,
          m = Math.sqrt(v),
          p = Math.sqrt(h),
          y = o * Math.tan((mu - Math.acos((v + h - j) / (2 * m * p))) / 2),
          w = y / p,
          S = y / m;
        Math.abs(w - 1) > un && this._append`L${t + w * f},${n + w * d}`,
          this._append`A${o},${o},0,0,${+(d * x > f * g)},${(this._x1 =
            t + S * u)},${(this._y1 = n + S * a)}`;
      }
  }
  arc(t, n, r, i, o, l) {
    if (((t = +t), (n = +n), (r = +r), (l = !!l), r < 0))
      throw new Error(`negative radius: ${r}`);
    let s = r * Math.cos(i),
      u = r * Math.sin(i),
      a = t + s,
      f = n + u,
      d = 1 ^ l,
      h = l ? i - o : o - i;
    this._x1 === null
      ? this._append`M${a},${f}`
      : (Math.abs(this._x1 - a) > un || Math.abs(this._y1 - f) > un) &&
        this._append`L${a},${f}`,
      r &&
        (h < 0 && (h = (h % yu) + yu),
        h > Rv
          ? this._append`A${r},${r},0,1,${d},${t - s},${
              n - u
            }A${r},${r},0,1,${d},${(this._x1 = a)},${(this._y1 = f)}`
          : h > un &&
            this._append`A${r},${r},0,${+(h >= mu)},${d},${(this._x1 =
              t + r * Math.cos(o))},${(this._y1 = n + r * Math.sin(o))}`);
  }
  rect(t, n, r, i) {
    this._append`M${(this._x0 = this._x1 = +t)},${(this._y0 = this._y1 =
      +n)}h${(r = +r)}v${+i}h${-r}Z`;
  }
  toString() {
    return this._;
  }
}
function bp(e) {
  let t = 3;
  return (
    (e.digits = function (n) {
      if (!arguments.length) return t;
      if (n == null) t = null;
      else {
        const r = Math.floor(n);
        if (!(r >= 0)) throw new RangeError(`invalid digits: ${n}`);
        t = r;
      }
      return e;
    }),
    () => new Iv(t)
  );
}
function Xp(e) {
  return typeof e == "object" && "length" in e ? e : Array.from(e);
}
function Kp(e) {
  this._context = e;
}
Kp.prototype = {
  areaStart: function () {
    this._line = 0;
  },
  areaEnd: function () {
    this._line = NaN;
  },
  lineStart: function () {
    this._point = 0;
  },
  lineEnd: function () {
    (this._line || (this._line !== 0 && this._point === 1)) &&
      this._context.closePath(),
      (this._line = 1 - this._line);
  },
  point: function (e, t) {
    switch (((e = +e), (t = +t), this._point)) {
      case 0:
        (this._point = 1),
          this._line ? this._context.lineTo(e, t) : this._context.moveTo(e, t);
        break;
      case 1:
        this._point = 2;
      default:
        this._context.lineTo(e, t);
        break;
    }
  },
};
function Zp(e) {
  return new Kp(e);
}
function Jp(e) {
  return e[0];
}
function qp(e) {
  return e[1];
}
function e0(e, t) {
  var n = he(!0),
    r = null,
    i = Zp,
    o = null,
    l = bp(s);
  (e = typeof e == "function" ? e : e === void 0 ? Jp : he(e)),
    (t = typeof t == "function" ? t : t === void 0 ? qp : he(t));
  function s(u) {
    var a,
      f = (u = Xp(u)).length,
      d,
      h = !1,
      x;
    for (r == null && (o = i((x = l()))), a = 0; a <= f; ++a)
      !(a < f && n((d = u[a]), a, u)) === h &&
        ((h = !h) ? o.lineStart() : o.lineEnd()),
        h && o.point(+e(d, a, u), +t(d, a, u));
    if (x) return (o = null), x + "" || null;
  }
  return (
    (s.x = function (u) {
      return arguments.length
        ? ((e = typeof u == "function" ? u : he(+u)), s)
        : e;
    }),
    (s.y = function (u) {
      return arguments.length
        ? ((t = typeof u == "function" ? u : he(+u)), s)
        : t;
    }),
    (s.defined = function (u) {
      return arguments.length
        ? ((n = typeof u == "function" ? u : he(!!u)), s)
        : n;
    }),
    (s.curve = function (u) {
      return arguments.length ? ((i = u), r != null && (o = i(r)), s) : i;
    }),
    (s.context = function (u) {
      return arguments.length
        ? (u == null ? (r = o = null) : (o = i((r = u))), s)
        : r;
    }),
    s
  );
}
function Uv(e, t, n) {
  var r = null,
    i = he(!0),
    o = null,
    l = Zp,
    s = null,
    u = bp(a);
  (e = typeof e == "function" ? e : e === void 0 ? Jp : he(+e)),
    (t = typeof t == "function" ? t : he(t === void 0 ? 0 : +t)),
    (n = typeof n == "function" ? n : n === void 0 ? qp : he(+n));
  function a(d) {
    var h,
      x,
      g,
      v = (d = Xp(d)).length,
      j,
      m = !1,
      p,
      y = new Array(v),
      w = new Array(v);
    for (o == null && (s = l((p = u()))), h = 0; h <= v; ++h) {
      if (!(h < v && i((j = d[h]), h, d)) === m)
        if ((m = !m)) (x = h), s.areaStart(), s.lineStart();
        else {
          for (s.lineEnd(), s.lineStart(), g = h - 1; g >= x; --g)
            s.point(y[g], w[g]);
          s.lineEnd(), s.areaEnd();
        }
      m &&
        ((y[h] = +e(j, h, d)),
        (w[h] = +t(j, h, d)),
        s.point(r ? +r(j, h, d) : y[h], n ? +n(j, h, d) : w[h]));
    }
    if (p) return (s = null), p + "" || null;
  }
  function f() {
    return e0().defined(i).curve(l).context(o);
  }
  return (
    (a.x = function (d) {
      return arguments.length
        ? ((e = typeof d == "function" ? d : he(+d)), (r = null), a)
        : e;
    }),
    (a.x0 = function (d) {
      return arguments.length
        ? ((e = typeof d == "function" ? d : he(+d)), a)
        : e;
    }),
    (a.x1 = function (d) {
      return arguments.length
        ? ((r = d == null ? null : typeof d == "function" ? d : he(+d)), a)
        : r;
    }),
    (a.y = function (d) {
      return arguments.length
        ? ((t = typeof d == "function" ? d : he(+d)), (n = null), a)
        : t;
    }),
    (a.y0 = function (d) {
      return arguments.length
        ? ((t = typeof d == "function" ? d : he(+d)), a)
        : t;
    }),
    (a.y1 = function (d) {
      return arguments.length
        ? ((n = d == null ? null : typeof d == "function" ? d : he(+d)), a)
        : n;
    }),
    (a.lineX0 = a.lineY0 =
      function () {
        return f().x(e).y(t);
      }),
    (a.lineY1 = function () {
      return f().x(e).y(n);
    }),
    (a.lineX1 = function () {
      return f().x(r).y(t);
    }),
    (a.defined = function (d) {
      return arguments.length
        ? ((i = typeof d == "function" ? d : he(!!d)), a)
        : i;
    }),
    (a.curve = function (d) {
      return arguments.length ? ((l = d), o != null && (s = l(o)), a) : l;
    }),
    (a.context = function (d) {
      return arguments.length
        ? (d == null ? (o = s = null) : (s = l((o = d))), a)
        : o;
    }),
    a
  );
}
function Hv(e) {
  const [, t] = Sr(),
    { colorScaler: n } = ue(),
    r = kr(e.id, "series"),
    { lineStyle: i = {}, displayMarkers: o = !1, hidden: l, ...s } = e,
    {
      xAxis: u = "x",
      yAxis: a = "y",
      xShift: f = "0",
      yShift: d = "0",
      pointLabel: h,
      displayErrorBars: x,
      label: g,
      markerStyle: v,
      markerShape: j,
    } = s,
    { xShift: m, yShift: p } = wl({ xAxis: u, yAxis: a, xShift: f, yShift: d }),
    y = gn({}, i, { id: r }),
    w = vl(r);
  if (
    (_.useEffect(() => {
      var k, E;
      if (!l)
        return (
          t({
            type: "ADD_LEGEND_LABEL",
            payload: {
              id: r,
              label: g,
              colorLine:
                ((k = y == null ? void 0 : y.stroke) === null || k === void 0
                  ? void 0
                  : k.toString()) || n(r),
              shape: {
                color:
                  ((E = v == null ? void 0 : v.fill) === null || E === void 0
                    ? void 0
                    : E.toString()) || n(r),
                figure: j || "circle",
                hidden: !o,
              },
            },
          }),
          () => t({ type: "REMOVE_LEGEND_LABEL", payload: { id: r } })
        );
    }, [
      l,
      n,
      o,
      r,
      t,
      y == null ? void 0 : y.stroke,
      g,
      j,
      v == null ? void 0 : v.fill,
    ]),
    l)
  )
    return null;
  const S = {
    id: r,
    data: e.data,
    xAxis: u,
    yAxis: a,
    lineStyle: y,
    transform: `translate(${m},${p})`,
  };
  return c.jsxs("g", {
    children: [
      w && c.jsx(Wv, { ...S }),
      c.jsx(Da, { ...s, hidden: !o && !h && !x, displayMarkers: o, id: r }),
    ],
  });
}
const kl = _.memo(Hv);
function Wv({
  id: e,
  data: t,
  xAxis: n,
  yAxis: r,
  lineStyle: i,
  transform: o,
}) {
  const { axisContext: l, colorScaler: s } = ue(),
    [u, a] = nt(l, n, r),
    f = s(e),
    d = _.useMemo(
      () =>
        u === void 0 || a === void 0
          ? null
          : e0()
              .x((g) => u(g.x))
              .y((g) => a(g.y))(t),
      [t, u, a]
    );
  if (!d) return null;
  const h = { stroke: f, strokeWidth: 2, ...i };
  return c.jsx("path", { transform: o, style: h, d, fill: "none" });
}
function Vv(e) {
  const { getY: t, xAxis: n = "x", id: r, ...i } = e,
    o = `~${kr(r, "series")}`,
    { axisContext: l, plotWidth: s, plotHeight: u } = ue(),
    a = l[n],
    f = 1,
    d = _.useMemo(() => {
      const h = [];
      if (a) {
        const g = (a ? a.position === "top" || a.position === "bottom" : !1)
            ? s
            : u,
          v = a.scale;
        for (let j = 0; j <= g; j += f) {
          const m = Vr(v.invert(j));
          h.push({ x: m, y: t(m) });
        }
        return h;
      }
      return [{ x: 0, y: 0 }];
    }, [a == null ? void 0 : a.domain[0], a == null ? void 0 : a.domain[1]]);
  return c.jsx(kl, { data: d, id: o, ...i });
}
function Bv(e) {
  const t = kr(e.id, "series"),
    [, n] = Sr(),
    { colorScaler: r } = ue(),
    {
      lineStyle: i = { fill: r(t), fillOpacity: 0.5 },
      hidden: o,
      xAxis: l = "x",
      yAxis: s = "y",
      data: u,
      label: a,
      xShift: f = "0",
      yShift: d = "0",
    } = e,
    { xShift: h, yShift: x } = wl({ xAxis: l, yAxis: s, xShift: f, yShift: d }),
    g = Di();
  _.useEffect(() => {
    const [m, p] = ti(u, (D) => D.x),
      [y, w] = ti(u, (D) => D.y1),
      [S, k] = ti(u, (D) => D.y2),
      E = { min: m, max: p, shift: f, axisId: l },
      P = { min: Math.min(y, S), max: Math.max(w, k), shift: d, axisId: s };
    return (
      g({ type: "addSeries", payload: { id: t, x: E, y: P, label: a } }),
      () => g({ type: "removeSeries", payload: { id: t } })
    );
  }, [g, t, u, l, s, a, f, d]);
  const v = vl(t);
  if (
    (_.useEffect(() => {
      if (!o)
        return (
          n({
            type: "ADD_LEGEND_LABEL",
            payload: {
              id: t,
              label: a,
              colorLine: i.stroke,
              range: { rangeColor: i.fill || "none" },
            },
          }),
          () => n({ type: "REMOVE_LEGEND_LABEL", payload: { id: t } })
        );
    }, [a, n, i.fill, i.stroke, t, o]),
    o)
  )
    return null;
  const j = {
    id: t,
    data: u,
    xAxis: l,
    yAxis: s,
    lineStyle: i,
    transform: `translate(${h},${x})`,
  };
  return v ? c.jsx(Qv, { ...j }) : null;
}
const Yv = _.memo(Bv);
function Qv({ data: e, xAxis: t, yAxis: n, lineStyle: r, transform: i }) {
  const { axisContext: o } = ue(),
    [l, s] = nt(o, t, n),
    u = _.useMemo(
      () =>
        l === void 0 || s === void 0
          ? null
          : Uv()
              .x((d) => l(d.x))
              .y0((d) => s(d.y1))
              .y1((d) => s(d.y2))(e),
      [e, l, s]
    );
  if (!u) return null;
  const a = { strokeWidth: 2, ...r };
  return c.jsx("path", { transform: i, style: a, d: u, fill: "none" });
}
function Gv(e) {
  return c.jsx(c.Fragment, { children: e.children });
}
function bv(e) {
  let t = null,
    n = null,
    r = null,
    i = null,
    o = [],
    l = null,
    s = null,
    u = [],
    a = [];
  for (let f of _.Children.toArray(e)) {
    if (typeof f != "object" || !_.isValidElement(f))
      throw (
        (console.error("Invalid Plot child:", f),
        new Error("invalid Plot child"))
      );
    if (
      f.type === Gv ||
      f.type === Vv ||
      f.type === kl ||
      f.type === Da ||
      f.type === Yv ||
      f.type === zv
    )
      u.push(f);
    else if (f.type === Ip) a.push(f);
    else if (f.type === xn)
      switch (f.props.position) {
        case "top": {
          if (t !== null) throw new Error("Plot can only have one top axis");
          t = f;
          break;
        }
        case "right": {
          if (n !== null) throw new Error("Plot can only have one right axis");
          n = f;
          break;
        }
        case "bottom": {
          if (r !== null) throw new Error("Plot can only have one bottom axis");
          r = f;
          break;
        }
        case "left": {
          if (i !== null) throw new Error("Plot can only have one left axis");
          i = f;
          break;
        }
        default:
          throw new Error("unreachable");
      }
    else if (f.type === Tv) {
      if (o.length === 2)
        throw new Error("Plot can have at most two parallel axes");
      o.push(f);
    } else if (f.type === Sl) {
      if (l !== null) throw new Error("Plot can only have one Heading element");
      l = f;
    } else if (f.type === Qp) {
      if (s !== null) throw new Error("Plot can only have one Legend element");
      s = f;
    } else
      throw (
        (console.error("Invalid Plot child:", f),
        new Error("invalid plot child"))
      );
  }
  !r && !t && (r = c.jsx(xn, { position: "bottom" })),
    !i && !n && (i = c.jsx(xn, { position: "left" }));
  for (const f of o) {
    const d = f.props.id;
    if ((t == null ? void 0 : t.props.id) === d) {
      if (r !== null) throw new Error("Plot can only have one bottom axis");
      r = f;
    }
    if ((n == null ? void 0 : n.props.id) === d) {
      if (i !== null) throw new Error("Plot can only have one left axis");
      i = f;
    }
    if ((r == null ? void 0 : r.props.id) === d) {
      if (t !== null) throw new Error("Plot can only have one top axis");
      t = f;
    }
    if ((i == null ? void 0 : i.props.id) === d) {
      if (n !== null) throw new Error("Plot can only have one right axis");
      n = f;
    }
  }
  return {
    topAxis: t,
    rightAxis: n,
    bottomAxis: r,
    leftAxis: i,
    heading: l,
    legend: s,
    series: u,
    annotations: a,
  };
}
function Xv({
  width: e,
  height: t,
  margin: n,
  axes: r,
  topAxisHeight: i,
  rightAxisWidth: o,
  bottomAxisHeight: l,
  leftAxisWidth: s,
  headingPosition: u,
  headingHeight: a,
  legendPosition: f,
  legendMargin: d,
  legendWidth: h,
  legendHeight: x,
}) {
  const { top: g = 0, right: v = 0, bottom: j = 0, left: m = 0 } = n;
  return _.useMemo(() => {
    const p = Kv(r),
      y = i - p.top,
      w = o - p.right,
      S = l - p.bottom,
      k = s - p.left;
    let E = e - m - k - v - w,
      P = t - g - a - y - j - S,
      D = m + k,
      O = g + y;
    u === "top" && (O += a);
    const N = x + d,
      W = h + d;
    let R = 0;
    switch (f) {
      case "top":
        (P -= N), (O += N), (R = y + d);
        break;
      case "right":
        (E -= W), (R = w + d);
        break;
      case "bottom":
        (P -= N), (R = S + d);
        break;
      case "left":
        (E -= W), (D += W), (R = k + d);
        break;
    }
    return {
      plotWidth: E,
      plotHeight: P,
      leftOffset: D,
      topOffset: O,
      legendOffset: R,
    };
  }, [e, t, g, v, j, m, i, o, l, s, u, a, f, d, h, x, r]);
}
function Kv(e) {
  const t = { top: 0, right: 0, bottom: 0, left: 0 };
  for (const n of Object.values(e)) t[n.position] = n.innerOffset;
  return t;
}
const Zv = new Set(["bottom", "top"]);
function Jv(e, t, n, r) {
  const { clientX: i, clientY: o, movementX: l, movementY: s } = e,
    { left: u, top: a } = r.getBoundingClientRect(),
    f = i - u,
    d = o - a,
    h = {},
    x = {},
    g = {},
    v = {};
  for (const j in t) {
    const { scale: m, clampInDomain: p, position: y, domain: w } = t[j];
    Zv.has(y)
      ? ((h[j] = Vr(m.invert(f))), (v[j] = Vr(m.invert(l)) - w[0]))
      : ((h[j] = Vr(m.invert(d))), (v[j] = Vr(m.invert(s)) - w[1])),
      (x[j] = p(h[j])),
      (g[j] = w);
  }
  return {
    event: e,
    coordinates: h,
    clampedCoordinates: x,
    movement: v,
    getClosest(j) {
      return qv(j, { x: f, y: d }, n, t);
    },
    domains: g,
  };
}
function qv(e, t, n, r) {
  let i = {};
  switch (e) {
    case "x": {
      for (const { id: o, x: l, data: s, label: u } of n)
        if (s) {
          const a = ls(s, t, (f, d) => {
            const { scale: h } = r[l.axisId],
              x = d[l.axisId];
            return Math.abs(h(f.x) - x);
          });
          i[o] = { point: a, label: u, axis: r[l.axisId] };
        }
      break;
    }
    case "y": {
      for (const { id: o, y: l, data: s, label: u } of n)
        if (s) {
          const a = ls(s, t, (f, d) => {
            const { scale: h } = r[l.axisId],
              x = d[l.axisId];
            return Math.abs(h(f.y) - x);
          });
          i[o] = { point: a, label: u, axis: r[l.axisId] };
        }
      break;
    }
    case "euclidean": {
      for (const { id: o, x: l, y: s, data: u, label: a } of n)
        if (u) {
          const f = ls(u, t, (d, h) => {
            const { scale: x } = r[l.axisId],
              { scale: g } = r[s.axisId],
              v = h[l.axisId],
              j = h[s.axisId];
            return Br([x(d.x), g(d.y)], [v, j]);
          });
          i[o] = {
            point: f,
            label: a,
            axis: { x: r[l.axisId], y: r[s.axisId] },
          };
        }
      break;
    }
    default:
      throw new Error(`Unknown distance name: ${e}`);
  }
  return i;
}
const ew = {
    pointerenter: "onPointerEnter",
    pointerdown: "onPointerDown",
    pointermove: "onPointerMove",
    pointerup: "onPointerUp",
    pointerleave: "onPointerLeave",
    click: "onClick",
    dblclick: "onDoubleClick",
    wheel: "onWheel",
  },
  Vf = [
    "pointerenter",
    "pointerdown",
    "pointermove",
    "pointerup",
    "pointerleave",
    "click",
    "dblclick",
    "wheel",
  ],
  us = ["pointermove", "pointerup"];
function tw({
  plotId: e,
  plotEvents: t,
  stateSeries: n,
  axisContext: r,
  plotHeight: i,
  plotWidth: o,
}) {
  const l = _.useRef(null),
    s = _.useRef({
      plotId: e,
      plotEvents: t,
      stateSeries: n,
      axisContext: r,
      plotHeight: i,
      plotWidth: o,
    });
  return (
    _.useEffect(() => {
      s.current = {
        plotId: e,
        plotEvents: t,
        stateSeries: n,
        axisContext: r,
        plotHeight: i,
        plotWidth: o,
      };
    }, [r, t, i, e, o, n]),
    _.useEffect(() => {
      const u = l.current;
      if (!u) return;
      function a(f) {
        if (f.type === "pointerdown")
          for (const h of us) window.addEventListener(h, a);
        else if (f.type === "pointerup")
          for (const h of us) window.removeEventListener(h, a);
        const d = Jv(f, s.current.axisContext, s.current.stateSeries, u);
        t.handleEvent(e, ew[f.type], d);
      }
      for (const f of Vf) u.addEventListener(f, a);
      return () => {
        for (const f of Vf) u.removeEventListener(f, a);
        for (const f of us) window.removeEventListener(f, a);
      };
    }, [e, t]),
    c.jsx("rect", { ref: l, width: o, height: i, style: { fillOpacity: 0 } })
  );
}
function nw(e) {
  const {
    width: t,
    height: n,
    svgStyle: r,
    svgId: i,
    svgClassName: o,
    topOffset: l,
    leftOffset: s,
    legendOffset: u,
    legend: a,
    legendRef: f,
    plotId: d,
    annotations: h,
    tracking: x,
  } = e;
  return c.jsxs("svg", {
    xmlns: "http://www.w3.org/2000/svg",
    width: t,
    height: n,
    style: r,
    id: i ? `${i}-annotations` : void 0,
    className: o,
    children: [
      c.jsxs("g", {
        transform: `translate(${s}, ${l})`,
        children: [
          c.jsx("g", {
            style: { clipPath: `url(#seriesViewportClip-${d})` },
            children: h,
          }),
          x,
        ],
      }),
      c.jsx("g", {
        transform: `translate(${s}, ${l})`,
        children: c.jsx(Yp.Provider, {
          value: u,
          children: c.jsx("g", { ref: f, children: a }),
        }),
      }),
    ],
  });
}
function Bf(e) {
  const { style: t } = e;
  return c.jsx("rect", {
    ...e,
    style: { fillOpacity: t != null && t.fill ? void 0 : 0, ...t },
  });
}
function rw(e) {
  const {
    width: t,
    height: n,
    svgStyle: r,
    svgId: i,
    svgClassName: o,
    plotViewportStyle: l,
    seriesViewportStyle: s,
    topOffset: u,
    leftOffset: a,
    plotWidth: f,
    plotHeight: d,
    plotId: h,
    series: x,
    topAxisRef: g,
    topAxis: v,
    rightAxisRef: j,
    rightAxis: m,
    bottomAxisRef: p,
    bottomAxis: y,
    leftAxisRef: w,
    leftAxis: S,
    headingRef: k,
    heading: E,
  } = e;
  return c.jsxs("svg", {
    xmlns: "http://www.w3.org/2000/svg",
    width: t,
    height: n,
    style: r,
    id: i,
    className: o,
    children: [
      c.jsx(Bf, { width: t, height: n, style: l }),
      c.jsxs("g", {
        transform: `translate(${a}, ${u})`,
        children: [
          c.jsx(Bf, { width: f, height: d, style: s }),
          c.jsx("clipPath", {
            id: `seriesViewportClip-${h}`,
            children: c.jsx("rect", { width: f, height: d }),
          }),
          c.jsx("g", {
            style: { clipPath: `url(#seriesViewportClip-${h})` },
            children: x,
          }),
          c.jsx(Jn.Provider, {
            value: g,
            children: c.jsx("g", { children: v }),
          }),
          c.jsx(Jn.Provider, {
            value: j,
            children: c.jsx("g", { children: m }),
          }),
          c.jsx(Jn.Provider, {
            value: p,
            children: c.jsx("g", { children: y }),
          }),
          c.jsx(Jn.Provider, {
            value: w,
            children: c.jsx("g", { children: S }),
          }),
        ],
      }),
      c.jsx("g", { ref: k, children: E }),
    ],
  });
}
const iw = Cp(A1),
  ow = {
    headingPosition: null,
    legendPosition: null,
    legendMargin: 0,
    series: [],
    axes: {},
  },
  lw = {
    overflow: "visible",
    fontFamily: "Arial, Helvetica, sans-serif",
    userSelect: "none",
    WebkitUserSelect: "none",
  },
  sw = { touchAction: "none" };
function t0(e) {
  const {
      width: t,
      height: n,
      colorScheme: r = s1,
      margin: i = {},
      svgStyle: o = {},
      svgId: l,
      svgClassName: s,
      plotViewportStyle: u = {},
      seriesViewportStyle: a = {},
      controllerId: f,
      children: d,
    } = e,
    h = kr(void 0, "plot"),
    [x, g] = _.useReducer(iw, ow, void 0),
    {
      series: v,
      annotations: j,
      topAxis: m,
      rightAxis: p,
      bottomAxis: y,
      leftAxis: w,
      heading: S,
      legend: k,
    } = bv(d);
  if (t === void 0) throw new Error("width is mandatory");
  if (n === void 0) throw new Error("height is mandatory");
  const E = U1({ controllerId: f });
  _.useEffect(() => {
    if (E) return E.registerPlot(h), () => E.unregisterPlot(h);
  }, [E, h]);
  const P = F1({ controllerId: f }),
    D = pt(),
    O = pt(),
    N = pt(),
    W = pt(),
    R = pt(),
    Q = pt(),
    {
      plotWidth: K,
      plotHeight: b,
      topOffset: le,
      leftOffset: T,
      legendOffset: L,
    } = Xv({
      width: t,
      height: n,
      margin: i,
      axes: x.axes,
      topAxisHeight: O.height,
      rightAxisWidth: N.width,
      bottomAxisHeight: W.height,
      leftAxisWidth: R.width,
      headingPosition: x.headingPosition,
      headingHeight: D.height,
      legendPosition: x.legendPosition,
      legendMargin: x.legendMargin,
      legendWidth: Q.width,
      legendHeight: Q.height,
    }),
    z = D1(x, P.axes, { plotWidth: K, plotHeight: b }),
    F = _.useMemo(() => x.series.map(({ id: nn }) => nn), [x.series]),
    X = _.useMemo(() => ha().range(r).domain(F), [r, F]),
    _e = _.useMemo(
      () => ({
        width: t,
        height: n,
        plotWidth: K,
        plotHeight: b,
        axisContext: z,
        colorScaler: X,
      }),
      [t, n, K, b, z, X]
    ),
    Ge = {
      ...lw,
      ...(E ? sw : null),
      ...o,
      position: "absolute",
      top: "0",
      left: "0",
    };
  return c.jsx(_p.Provider, {
    value: _e,
    children: c.jsx(Lp.Provider, {
      value: g,
      children: c.jsx(M1, {
        children: c.jsxs("div", {
          style: { position: "relative", width: t, height: n },
          children: [
            c.jsx(rw, {
              width: t,
              height: n,
              svgStyle: Ge,
              svgId: l,
              svgClassName: s,
              plotViewportStyle: u,
              seriesViewportStyle: a,
              topOffset: le,
              leftOffset: T,
              plotWidth: K,
              plotHeight: b,
              plotId: h,
              series: v,
              topAxisRef: O.ref,
              topAxis: m,
              rightAxisRef: N.ref,
              rightAxis: p,
              bottomAxisRef: W.ref,
              bottomAxis: y,
              leftAxisRef: R.ref,
              leftAxis: w,
              headingRef: D.ref,
              heading: S,
            }),
            c.jsx(nw, {
              width: t,
              height: n,
              svgStyle: Ge,
              svgId: l,
              svgClassName: s,
              topOffset: le,
              leftOffset: T,
              legendOffset: L,
              legend: k,
              legendRef: Q.ref,
              plotId: h,
              annotations: j,
              tracking: E
                ? c.jsx(tw, {
                    plotId: h,
                    plotEvents: E,
                    stateSeries: x.series,
                    axisContext: z,
                    plotWidth: K,
                    plotHeight: b,
                  })
                : null,
            }),
          ],
        }),
      }),
    }),
  });
}
const no = 1,
  Yf = 0.1;
function uw(e) {
  const t = e.robot.odometry.pose,
    n = e.odometryPlot,
    r = t.x + Yf * Math.cos(t.theta),
    i = t.y + Yf * Math.sin(t.theta);
  return c.jsxs(t0, {
    width: 500,
    height: 500,
    margin: { top: 20, right: 20, bottom: 20, left: 20 },
    children: [
      c.jsx(Sl, { title: "Robot position in xy plane" }),
      c.jsx(kl, {
        displayMarkers: !0,
        markerShape: "circle",
        markerSize: 3,
        data: n.series[0],
      }),
      c.jsxs(Ip, {
        children: [
          c.jsx(Hf.Circle, { x: t.x, y: t.y, r: 0.03, color: "green" }),
          c.jsx(Hf.Line, {
            x1: t.x,
            y1: t.y,
            x2: r,
            y2: i,
            color: "blue",
            strokeWidth: 3,
          }),
        ],
      }),
      c.jsx(xn, {
        id: "x",
        position: "bottom",
        label: "x [m]",
        displayPrimaryGridLines: !0,
        min: -no,
        max: no,
      }),
      c.jsx(xn, {
        id: "y",
        position: "left",
        label: "y [m]",
        displayPrimaryGridLines: !0,
        min: -no,
        max: no,
      }),
    ],
  });
}
const as = 3;
function aw(e) {
  const t = e.robot.odometry;
  return c.jsx("div", {
    children: c.jsxs("table", {
      children: [
        c.jsxs("tr", {
          children: [
            c.jsx("th", { align: "left", children: " Variable " }),
            c.jsx("th", { children: " Value" }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "x [m]" }),
            c.jsx("td", { align: "center", children: t.pose.x.toFixed(as) }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "y [m]" }),
            c.jsx("td", { align: "center", children: t.pose.y.toFixed(as) }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "theta [rad]" }),
            c.jsx("td", {
              align: "center",
              children: t.pose.theta.toFixed(as),
            }),
          ],
        }),
      ],
    }),
  });
}
const zr = 3;
function Qf(e) {
  const { controller: t, buttons: n } = e;
  return c.jsx("div", {
    children: c.jsxs("table", {
      children: [
        c.jsxs("tr", {
          children: [
            c.jsx("th", { align: "left", children: " Parameter " }),
            c.jsx("th", { children: " Value" }),
            c.jsx("th", { children: " Set value" }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "k_p" }),
            c.jsx("td", { align: "center", children: t.kp.toFixed(zr) }),
            c.jsx("td", { align: "left", children: Or(n.p) }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "k_i" }),
            c.jsx("td", { align: "center", children: t.ki.toFixed(zr) }),
            c.jsx("td", { align: "left", children: Or(n.i) }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "k_d" }),
            c.jsx("td", { align: "center", children: t.kd.toFixed(zr) }),
            c.jsx("td", { align: "left", children: Or(n.d) }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "Target value" }),
            c.jsx("td", { align: "center", children: t.target.toFixed(zr) }),
            c.jsx("td", { align: "left", children: Or(n.target) }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "Current value" }),
            c.jsx("td", { align: "center", children: t.current.toFixed(zr) }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "Mode" }),
            c.jsx("td", {
              align: "center",
              children: t.mode ? "Enabled" : "Disabled",
            }),
            c.jsx("td", { align: "left", children: Or(n.mode) }),
          ],
        }),
      ],
    }),
  });
}
function Or(e) {
  return e.values.map((t, n) =>
    c.jsx(H, { command: `${e.code}${t}`, label: e.labels[n] }, n)
  );
}
const cw = [
  "Stop",
  "Same PWM",
  "",
  "",
  "",
  "",
  "Stop when obstacle",
  "",
  "Wheels speed control",
  "Robot speed control",
];
function fw() {
  return c.jsx("div", {
    children: c.jsxs("table", {
      children: [
        c.jsxs("tr", {
          children: [
            c.jsx("th", { align: "left", children: " Select mode " }),
            c.jsx("th", {
              align: "left",
              children: " Set speeds and other params",
            }),
            c.jsx("th", { align: "left", children: "Unit" }),
            c.jsx("th", { align: "center", children: "Serial param" }),
          ],
        }),
        c.jsx("tr", {
          children: c.jsx("td", {
            align: "left",
            children: c.jsx(H, { command: "T0", label: "Stop" }),
          }),
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", {
              align: "left",
              children: c.jsx(H, { command: "T1", label: "Same PWM" }),
            }),
            c.jsxs("td", {
              align: "left",
              children: [
                c.jsx(H, { command: "P0", label: "0" }),
                c.jsx(H, { command: "P50", label: "50" }),
                c.jsx(H, { command: "P100", label: "100" }),
                c.jsx(H, { command: "P150", label: "150" }),
              ],
            }),
            c.jsx("td", { align: "left", children: "[-]" }),
            c.jsx("td", { align: "center", children: "P" }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", {
              align: "left",
              children: c.jsx(H, {
                command: "T8",
                label: "Wheels speed control",
              }),
            }),
            c.jsxs("td", {
              align: "left",
              children: [
                c.jsx(H, { command: "Q0", label: "0" }),
                c.jsx(H, { command: "Q100", label: "100" }),
                c.jsx(H, { command: "Q300", label: "300" }),
                c.jsx(H, { command: "Q500", label: "500" }),
              ],
            }),
            c.jsx("td", { align: "left", children: "[rpm]" }),
            c.jsx("td", { align: "center", children: "Q" }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", {
              align: "left",
              children: c.jsx(H, {
                command: "T9",
                label: "Robot speed control",
              }),
            }),
            c.jsxs("td", {
              align: "left",
              children: [
                c.jsx(H, { command: "R0", label: "0" }),
                c.jsx(H, { command: "R100", label: "0.1" }),
                c.jsx(H, { command: "R300", label: "0.3" }),
                c.jsx(H, { command: "R500", label: "0.5" }),
                c.jsx("br", {}),
                c.jsx(H, { command: "S0", label: "0" }),
                c.jsx(H, { command: "S45", label: "45" }),
                c.jsx(H, { command: "S90", label: "90" }),
                c.jsx(H, { command: "S180", label: "180" }),
              ],
            }),
            c.jsxs("td", {
              align: "left",
              children: ["[m/s]", c.jsx("br", {}), "[deg/s]"],
            }),
            c.jsxs("td", {
              align: "center",
              children: ["R", c.jsx("br", {}), "S"],
            }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", {
              align: "left",
              children: c.jsx(H, {
                command: "T6",
                label: "Stop when obstacle",
              }),
            }),
            c.jsxs("td", {
              align: "left",
              children: [
                c.jsx(H, { command: "Q0", label: "0" }),
                c.jsx(H, { command: "Q100", label: "100" }),
                c.jsx(H, { command: "Q300", label: "300" }),
                c.jsx(H, { command: "Q500", label: "500" }),
                c.jsx(H, { command: "Q1200", label: "1200" }),
                c.jsx("br", {}),
                c.jsx(H, { command: "V0", label: "0" }),
                c.jsx(H, { command: "100", label: "100" }),
                c.jsx(H, { command: "V200", label: "200" }),
                c.jsx(H, { command: "V500", label: "500" }),
                c.jsx(H, { command: "V1000", label: "1000" }),
              ],
            }),
            c.jsxs("td", {
              align: "left",
              children: ["[rpm]", c.jsx("br", {}), "[mm]"],
            }),
            c.jsxs("td", {
              align: "center",
              children: ["Q", c.jsx("br", {}), "V (distance)"],
            }),
          ],
        }),
      ],
    }),
  });
}
function Rr(e) {
  const {
    series: t,
    labels: n,
    title: r = "My plot",
    xLabel: i = "x",
    yLabel: o = "y",
    yLimit: l = 100,
    legendPosition: s = "bottom",
    plotHeight: u = 300,
    plotWidth: a = 600,
  } = e;
  return c.jsxs(t0, {
    width: a,
    height: u,
    margin: { top: 20, right: 20, bottom: 20, left: 20 },
    children: [
      c.jsx(Qp, { position: s }),
      c.jsx(Sl, { title: r }),
      t.map((f, d) =>
        c.jsx(
          kl,
          {
            displayMarkers: !0,
            markerShape: "circle",
            markerSize: 5,
            data: f,
            label: n[d],
          },
          d
        )
      ),
      c.jsx(xn, {
        id: "x",
        position: "bottom",
        label: i,
        displayPrimaryGridLines: !0,
      }),
      c.jsx(xn, {
        id: "y",
        position: "left",
        label: o,
        displayPrimaryGridLines: !0,
      }),
    ],
  });
}
const Gf = 0;
function dw(e) {
  return c.jsx("div", {
    children: c.jsxs("table", {
      children: [
        c.jsxs("tr", {
          children: [
            c.jsx("th", { align: "left", children: " Wheel " }),
            c.jsx("th", { children: " Speed [rpm]" }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "Left" }),
            c.jsx("td", {
              align: "center",
              children: e.robot.leftMotor.speed.toFixed(Gf),
            }),
          ],
        }),
        c.jsxs("tr", {
          children: [
            c.jsx("td", { children: "Right" }),
            c.jsx("td", {
              align: "center",
              children: e.robot.rightMotor.speed.toFixed(Gf),
            }),
          ],
        }),
      ],
    }),
  });
}
const hw = {
    p: {
      code: "BG",
      values: [0, 100, 200, 300],
      labels: ["0", "10", "20", "30"],
      unit: "[-]",
    },
    i: {
      code: "BH",
      values: [0, 20, 40, 100],
      labels: ["0", "2", "4", "10"],
      unit: "[-]",
    },
    d: {
      code: "BI",
      values: [0, 10, 50, 100],
      labels: ["0", "1", "5", "10"],
      unit: "[-]",
    },
    target: {
      code: "R",
      values: [0, 100, 200, 300, 400],
      labels: ["0", "0.1", "0.2", "0.3", "0.4"],
      unit: "[m/s]",
    },
    mode: { code: "BA", values: [0, 1], labels: ["Off", "On"], unit: "[-]" },
  },
  pw = {
    p: {
      code: "BJ",
      values: [0, 100, 300, 1e3],
      labels: ["0", "1", "3", "10"],
      unit: "[-]",
    },
    i: {
      code: "BK",
      values: [0, 10, 20, 100],
      labels: ["0", "0.1", "0.2", "1.0"],
      unit: "[-]",
    },
    d: {
      code: "BL",
      values: [0, 1, 5, 10],
      labels: ["0", "0.01", "0.05", "0.1"],
      unit: "[-]",
    },
    target: {
      code: "S",
      values: [0, 45, 90, 180],
      labels: ["0", "45", "90", "180"],
      unit: "[deg/s]",
    },
    mode: { code: "BB", values: [0, 1], labels: ["Off", "On"], unit: "[-]" },
  },
  zi = 1e6;
function mw(e, t) {
  const { distancePlot: n } = e,
    { distances: r } = t,
    i = t.odometry.time,
    o = jr(n.series, i, r, { xFactor: zi, maxDataLength: Ke });
  return { ...n, series: o };
}
function yw(e, t) {
  const { odometryPlot: n } = e,
    { odometry: r } = t,
    i = jr(n.series, r.pose.x, [r.pose.y], { maxDataLength: Ke });
  return { ...n, series: i };
}
function gw(e, t) {
  const { linearSpeedControllerPlot: n } = e,
    { controllers: r } = t,
    i = t.odometry.time,
    o = [r.v.target, r.v.current],
    l = jr(n.series, i, o, { xFactor: zi, maxDataLength: Ke });
  return { ...n, series: l };
}
function xw(e, t) {
  const { angularSpeedControllerPlot: n } = e,
    { controllers: r } = t,
    i = t.odometry.time,
    o = [r.omega.target, r.omega.current],
    l = jr(n.series, i, o, { xFactor: zi, maxDataLength: Ke });
  return { ...n, series: l };
}
function vw(e, t) {
  const { commandsPlot: n } = e,
    { leftMotor: r, rightMotor: i } = t,
    o = t.odometry.time,
    l = [r.command, i.command],
    s = jr(n.series, o, l, { xFactor: zi, maxDataLength: Ke });
  return { ...n, series: s };
}
function ww(e, t) {
  const { wheelSpeedsPlot: n } = e,
    { leftMotor: r, rightMotor: i } = t,
    o = t.odometry.time,
    l = [r.speed, i.speed],
    s = jr(n.series, o, l, { xFactor: zi, maxDataLength: Ke });
  return { ...n, series: s };
}
function jr(e, t, n, r = {}) {
  const { xFactor: i = 1, yFactor: o = 1, maxDataLength: l = 100 } = r,
    s = e,
    u = s.length;
  for (let a = 0; a < u; a++) {
    let f = n[a];
    a > 2 && (f = -n[a]),
      (s[a] = [...s[a], { x: t / i, y: f / o }].slice(1, l + 1));
  }
  return s;
}
const Ke = 100,
  bf = 5;
function Sw() {
  const e = { target: 0, current: 0, kp: 0, ki: 0, kd: 0, mode: !0 };
  return {
    robot: {
      navigation: { mode: 0 },
      imu: {
        acceleration: { x: 0, y: 0, z: 0 },
        rotation: { x: 0, y: 0, z: 0 },
      },
      distances: new Array(bf).fill(0),
      odometry: { pose: { x: 0, y: 0, theta: 0 }, time: 0 },
      controllers: { v: e, omega: e, commands: { left: 0, right: 0 } },
      leftMotor: { command: 0, speed: 0 },
      rightMotor: { command: 0, speed: 0 },
    },
    maze: {
      cellSize: 50,
      cellValues: [
        { x: 0, y: 0, label: "A" },
        { x: 1, y: 0, label: "B" },
        { x: 0, y: 1, label: "C" },
        { x: 1, y: 1, label: "D" },
      ],
    },
    distancePlot: {
      series: Rn(bf, Ke),
      labels: ["Left", "Front-left", "Front", "Front-right", "Right"],
      xLabel: "Time [s]",
      yLabel: "Distance [mm]",
      title: "Distance sensors",
      yLimit: 1e3,
      legendPosition: "right",
      plotWidth: 800,
    },
    odometryPlot: { series: Rn(1, Ke), labels: ["Position"] },
    linearSpeedControllerPlot: {
      series: Rn(2, Ke),
      labels: ["Target speed", "Current speed"],
      xLabel: "Time [s]",
      yLimit: 0.5,
      yLabel: "v [m/s]",
      title: "Linear speed controller",
    },
    angularSpeedControllerPlot: {
      series: Rn(2, Ke),
      labels: ["Target speed", "Current speed"],
      xLabel: "Time [s]",
      yLabel: "omega [rad/s]",
      title: "Angular speed controller",
      yLimit: 5,
    },
    commandsPlot: {
      series: Rn(2, Ke),
      labels: ["Left", "Right"],
      xLabel: "Time [s]",
      yLabel: "Command [-]",
      title: "PWM commands applied to the motors",
      yLimit: 260,
    },
    wheelSpeedsPlot: {
      series: Rn(2, Ke),
      labels: ["Left", "Right"],
      xLabel: "Time [s]",
      yLabel: "Speed [rpm]",
      title: "Wheel speeds",
      yLimit: 600,
    },
  };
}
function Rn(e, t) {
  return new Array(e).fill(0).map(() => new Array(t).fill({ x: 0, y: 0 }));
}
function kw(e, t) {
  const n = JSON.parse(t);
  return {
    robot: n,
    maze: e.maze,
    distancePlot: mw(e, n),
    odometryPlot: yw(e, n),
    linearSpeedControllerPlot: gw(e, n),
    angularSpeedControllerPlot: xw(e, n),
    commandsPlot: vw(e, n),
    wheelSpeedsPlot: ww(e, n),
  };
}
const Yr = "";
function jw() {
  const [e, t] = _.useState(Sw()),
    [n, r] = _.useState(""),
    [i, o] = _.useState("");
  return (
    _.useEffect(() => {
      const l = new EventSource(`${Yr}/events`);
      return (
        (l.onopen = () => {}),
        (l.onerror = (s) => {
          console.log("ERROR!", s);
        }),
        (l.onmessage = (s) => {}),
        l.addEventListener(
          "state",
          (s) => {
            const u = kw(e, s.data);
            t(u);
          },
          !1
        ),
        () => l.close()
      );
    }, [e, Yr]),
    c.jsxs("div", {
      style: { overflow: "clip" },
      children: [
        c.jsxs("div", {
          children: [
            "Command:",
            c.jsx("input", {
              type: "text",
              value: n,
              onChange: (l) => r(l.target.value),
              onKeyDown: async (l) => {
                l.key === "Enter" && (await gu(Yr, n, o));
              },
            }),
            c.jsx("button", { onClick: () => gu(Yr, n, o), children: "Send" }),
            c.jsx(H, { command: "h", label: "Help", setResponse: o }),
            c.jsx(H, { command: "ps", label: "State", setResponse: o }),
            c.jsx(H, { command: "s", label: "Settings", setResponse: o }),
          ],
        }),
        c.jsx("div", {
          children: c.jsx("textarea", { cols: 100, rows: 10, value: i }),
        }),
        c.jsx("h2", { children: "Accelerometer data" }),
        c.jsx(Ay, { ...e.robot.imu }),
        c.jsx("h2", { children: "Distance sensors" }),
        c.jsxs("div", {
          style: { display: "flex" },
          children: [
            c.jsx("div", { style: { flex: 1 }, children: c.jsx($y, { ...e }) }),
            c.jsx("div", {
              style: { flex: 1 },
              children: c.jsx(Rr, { ...e.distancePlot }),
            }),
          ],
        }),
        c.jsx("h2", { children: "Odometry" }),
        c.jsxs("div", {
          style: { display: "flex" },
          children: [
            c.jsxs("div", {
              style: { flex: 1 },
              children: [
                c.jsx("div", {
                  children: c.jsx(H, {
                    command: "Y1",
                    label: "Reset odometry",
                  }),
                }),
                c.jsx(aw, { ...e }),
              ],
            }),
            c.jsx("div", { style: { flex: 1 }, children: c.jsx(uw, { ...e }) }),
          ],
        }),
        c.jsx("h2", { children: "Robot control mode (T)" }),
        c.jsxs("div", {
          children: ["Current robot mode: ", cw[e.robot.navigation.mode]],
        }),
        c.jsx(fw, {}),
        c.jsx("h2", { children: "Robot speed controllers" }),
        c.jsx("h3", { children: "Linear speed" }),
        c.jsxs("div", {
          style: { display: "flex" },
          children: [
            c.jsx("div", {
              style: { flex: 1 },
              children: c.jsx(Qf, {
                controller: e.robot.controllers.v,
                buttons: hw,
              }),
            }),
            c.jsx("div", {
              style: { flex: 1 },
              children: c.jsx(Rr, { ...e.linearSpeedControllerPlot }),
            }),
          ],
        }),
        c.jsx("h3", { children: "Angular speed" }),
        c.jsxs("div", {
          style: { display: "flex" },
          children: [
            c.jsx("div", {
              style: { flex: 1 },
              children: c.jsx(Qf, {
                controller: e.robot.controllers.omega,
                buttons: pw,
              }),
            }),
            c.jsx("div", {
              style: { flex: 1 },
              children: c.jsx(Rr, { ...e.angularSpeedControllerPlot }),
            }),
          ],
        }),
        c.jsx("h2", { children: "Motors PWM commands" }),
        c.jsxs("div", {
          style: { display: "flex" },
          children: [
            c.jsx("div", { style: { flex: 1 }, children: c.jsx(Ny, { ...e }) }),
            c.jsx("div", {
              style: { flex: 1 },
              children: c.jsx(Rr, { ...e.commandsPlot }),
            }),
          ],
        }),
        c.jsx("h2", { children: "Wheels speed" }),
        c.jsxs("div", {
          style: { display: "flex" },
          children: [
            c.jsx("div", { style: { flex: 1 }, children: c.jsx(dw, { ...e }) }),
            c.jsx("div", {
              style: { flex: 1 },
              children: c.jsx(Rr, { ...e.wheelSpeedsPlot }),
            }),
          ],
        }),
      ],
    })
  );
}
async function gu(e, t, n) {
  const i = await (await fetch(`${e}/command?value=${t}`)).text();
  n == null || n(i);
}
function H(e) {
  const { command: t, label: n, setResponse: r } = e;
  return c.jsx("button", { onClick: () => gu(Yr, t, r), children: n || t });
}
cs.createRoot(document.getElementById("root")).render(
  c.jsxs(v0.StrictMode, {
    children: [c.jsx("h1", { children: "Algernon debug page" }), c.jsx(jw, {})],
  })
);
