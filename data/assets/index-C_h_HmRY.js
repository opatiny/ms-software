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
function n0(e) {
  return e && e.__esModule && Object.prototype.hasOwnProperty.call(e, "default")
    ? e.default
    : e;
}
var Xf = { exports: {} },
  Jo = {},
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
 */ var Pi = Symbol.for("react.element"),
  r0 = Symbol.for("react.portal"),
  i0 = Symbol.for("react.fragment"),
  o0 = Symbol.for("react.strict_mode"),
  l0 = Symbol.for("react.profiler"),
  s0 = Symbol.for("react.provider"),
  u0 = Symbol.for("react.context"),
  a0 = Symbol.for("react.forward_ref"),
  c0 = Symbol.for("react.suspense"),
  f0 = Symbol.for("react.memo"),
  d0 = Symbol.for("react.lazy"),
  za = Symbol.iterator;
function h0(e) {
  return e === null || typeof e != "object"
    ? null
    : ((e = (za && e[za]) || e["@@iterator"]),
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
  qf = Object.assign,
  Jf = {};
function yr(e, t, n) {
  (this.props = e),
    (this.context = t),
    (this.refs = Jf),
    (this.updater = n || Zf);
}
yr.prototype.isReactComponent = {};
yr.prototype.setState = function (e, t) {
  if (typeof e != "object" && typeof e != "function" && e != null)
    throw Error(
      "setState(...): takes an object of state variables to update or a function which returns an object of state variables."
    );
  this.updater.enqueueSetState(this, e, t, "setState");
};
yr.prototype.forceUpdate = function (e) {
  this.updater.enqueueForceUpdate(this, e, "forceUpdate");
};
function bf() {}
bf.prototype = yr.prototype;
function gu(e, t, n) {
  (this.props = e),
    (this.context = t),
    (this.refs = Jf),
    (this.updater = n || Zf);
}
var xu = (gu.prototype = new bf());
xu.constructor = gu;
qf(xu, yr.prototype);
xu.isPureReactComponent = !0;
var Oa = Array.isArray,
  ed = Object.prototype.hasOwnProperty,
  vu = { current: null },
  td = { key: !0, ref: !0, __self: !0, __source: !0 };
function nd(e, t, n) {
  var r,
    i = {},
    o = null,
    l = null;
  if (t != null)
    for (r in (t.ref !== void 0 && (l = t.ref),
    t.key !== void 0 && (o = "" + t.key),
    t))
      ed.call(t, r) && !td.hasOwnProperty(r) && (i[r] = t[r]);
  var s = arguments.length - 2;
  if (s === 1) i.children = n;
  else if (1 < s) {
    for (var u = Array(s), a = 0; a < s; a++) u[a] = arguments[a + 2];
    i.children = u;
  }
  if (e && e.defaultProps)
    for (r in ((s = e.defaultProps), s)) i[r] === void 0 && (i[r] = s[r]);
  return {
    $$typeof: Pi,
    type: e,
    key: o,
    ref: l,
    props: i,
    _owner: vu.current,
  };
}
function p0(e, t) {
  return {
    $$typeof: Pi,
    type: e.type,
    key: t,
    ref: e.ref,
    props: e.props,
    _owner: e._owner,
  };
}
function wu(e) {
  return typeof e == "object" && e !== null && e.$$typeof === Pi;
}
function m0(e) {
  var t = { "=": "=0", ":": "=2" };
  return (
    "$" +
    e.replace(/[=:]/g, function (n) {
      return t[n];
    })
  );
}
var Ra = /\/+/g;
function kl(e, t) {
  return typeof e == "object" && e !== null && e.key != null
    ? m0("" + e.key)
    : t.toString(36);
}
function to(e, t, n, r, i) {
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
          case Pi:
          case r0:
            l = !0;
        }
    }
  if (l)
    return (
      (l = e),
      (i = i(l)),
      (e = r === "" ? "." + kl(l, 0) : r),
      Oa(i)
        ? ((n = ""),
          e != null && (n = e.replace(Ra, "$&/") + "/"),
          to(i, t, n, "", function (a) {
            return a;
          }))
        : i != null &&
          (wu(i) &&
            (i = p0(
              i,
              n +
                (!i.key || (l && l.key === i.key)
                  ? ""
                  : ("" + i.key).replace(Ra, "$&/") + "/") +
                e
            )),
          t.push(i)),
      1
    );
  if (((l = 0), (r = r === "" ? "." : r + ":"), Oa(e)))
    for (var s = 0; s < e.length; s++) {
      o = e[s];
      var u = r + kl(o, s);
      l += to(o, t, n, u, i);
    }
  else if (((u = h0(e)), typeof u == "function"))
    for (e = u.call(e), s = 0; !(o = e.next()).done; )
      (o = o.value), (u = r + kl(o, s++)), (l += to(o, t, n, u, i));
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
function Di(e, t, n) {
  if (e == null) return e;
  var r = [],
    i = 0;
  return (
    to(e, r, "", "", function (o) {
      return t.call(n, o, i++);
    }),
    r
  );
}
function y0(e) {
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
  no = { transition: null },
  g0 = {
    ReactCurrentDispatcher: Me,
    ReactCurrentBatchConfig: no,
    ReactCurrentOwner: vu,
  };
function rd() {
  throw Error("act(...) is not supported in production builds of React.");
}
I.Children = {
  map: Di,
  forEach: function (e, t, n) {
    Di(
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
      Di(e, function () {
        t++;
      }),
      t
    );
  },
  toArray: function (e) {
    return (
      Di(e, function (t) {
        return t;
      }) || []
    );
  },
  only: function (e) {
    if (!wu(e))
      throw Error(
        "React.Children.only expected to receive a single React element child."
      );
    return e;
  },
};
I.Component = yr;
I.Fragment = i0;
I.Profiler = l0;
I.PureComponent = gu;
I.StrictMode = o0;
I.Suspense = c0;
I.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED = g0;
I.act = rd;
I.cloneElement = function (e, t, n) {
  if (e == null)
    throw Error(
      "React.cloneElement(...): The argument must be a React element, but you passed " +
        e +
        "."
    );
  var r = qf({}, e.props),
    i = e.key,
    o = e.ref,
    l = e._owner;
  if (t != null) {
    if (
      (t.ref !== void 0 && ((o = t.ref), (l = vu.current)),
      t.key !== void 0 && (i = "" + t.key),
      e.type && e.type.defaultProps)
    )
      var s = e.type.defaultProps;
    for (u in t)
      ed.call(t, u) &&
        !td.hasOwnProperty(u) &&
        (r[u] = t[u] === void 0 && s !== void 0 ? s[u] : t[u]);
  }
  var u = arguments.length - 2;
  if (u === 1) r.children = n;
  else if (1 < u) {
    s = Array(u);
    for (var a = 0; a < u; a++) s[a] = arguments[a + 2];
    r.children = s;
  }
  return { $$typeof: Pi, type: e.type, key: i, ref: o, props: r, _owner: l };
};
I.createContext = function (e) {
  return (
    (e = {
      $$typeof: u0,
      _currentValue: e,
      _currentValue2: e,
      _threadCount: 0,
      Provider: null,
      Consumer: null,
      _defaultValue: null,
      _globalName: null,
    }),
    (e.Provider = { $$typeof: s0, _context: e }),
    (e.Consumer = e)
  );
};
I.createElement = nd;
I.createFactory = function (e) {
  var t = nd.bind(null, e);
  return (t.type = e), t;
};
I.createRef = function () {
  return { current: null };
};
I.forwardRef = function (e) {
  return { $$typeof: a0, render: e };
};
I.isValidElement = wu;
I.lazy = function (e) {
  return { $$typeof: d0, _payload: { _status: -1, _result: e }, _init: y0 };
};
I.memo = function (e, t) {
  return { $$typeof: f0, type: e, compare: t === void 0 ? null : t };
};
I.startTransition = function (e) {
  var t = no.transition;
  no.transition = {};
  try {
    e();
  } finally {
    no.transition = t;
  }
};
I.unstable_act = rd;
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
const x0 = n0(_);
/**
 * @license React
 * react-jsx-runtime.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */ var v0 = _,
  w0 = Symbol.for("react.element"),
  k0 = Symbol.for("react.fragment"),
  S0 = Object.prototype.hasOwnProperty,
  E0 = v0.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED.ReactCurrentOwner,
  P0 = { key: !0, ref: !0, __self: !0, __source: !0 };
function id(e, t, n) {
  var r,
    i = {},
    o = null,
    l = null;
  n !== void 0 && (o = "" + n),
    t.key !== void 0 && (o = "" + t.key),
    t.ref !== void 0 && (l = t.ref);
  for (r in t) S0.call(t, r) && !P0.hasOwnProperty(r) && (i[r] = t[r]);
  if (e && e.defaultProps)
    for (r in ((t = e.defaultProps), t)) i[r] === void 0 && (i[r] = t[r]);
  return {
    $$typeof: w0,
    type: e,
    key: o,
    ref: l,
    props: i,
    _owner: E0.current,
  };
}
Jo.Fragment = k0;
Jo.jsx = id;
Jo.jsxs = id;
Xf.exports = Jo;
var h = Xf.exports,
  as = {},
  od = { exports: {} },
  Be = {},
  ld = { exports: {} },
  sd = {};
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
      e: for (var F = 0, X = T.length, Te = X >>> 1; F < Te; ) {
        var Ge = 2 * (F + 1) - 1,
          tn = T[Ge],
          vt = Ge + 1,
          Nn = T[vt];
        if (0 > i(tn, z))
          vt < X && 0 > i(Nn, tn)
            ? ((T[F] = Nn), (T[vt] = z), (F = vt))
            : ((T[F] = tn), (T[Ge] = z), (F = Ge));
        else if (vt < X && 0 > i(Nn, z)) (T[F] = Nn), (T[vt] = z), (F = vt);
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
    c = 1,
    f = null,
    d = 3,
    x = !1,
    g = !1,
    v = !1,
    E = typeof setTimeout == "function" ? setTimeout : null,
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
      if (n(u) !== null) (g = !0), Q(k);
      else {
        var L = n(a);
        L !== null && oe(w, L.startTime - T);
      }
  }
  function k(T, L) {
    (g = !1), v && ((v = !1), m(C), (C = -1)), (x = !0);
    var z = d;
    try {
      for (
        y(L), f = n(u);
        f !== null && (!(f.expirationTime > L) || (T && !N()));

      ) {
        var F = f.callback;
        if (typeof F == "function") {
          (f.callback = null), (d = f.priorityLevel);
          var X = F(f.expirationTime <= L);
          (L = e.unstable_now()),
            typeof X == "function" ? (f.callback = X) : f === n(u) && r(u),
            y(L);
        } else r(u);
        f = n(u);
      }
      if (f !== null) var Te = !0;
      else {
        var Ge = n(a);
        Ge !== null && oe(w, Ge.startTime - L), (Te = !1);
      }
      return Te;
    } finally {
      (f = null), (d = z), (x = !1);
    }
  }
  var S = !1,
    P = null,
    C = -1,
    D = 5,
    O = -1;
  function N() {
    return !(e.unstable_now() - O < D);
  }
  function H() {
    if (P !== null) {
      var T = e.unstable_now();
      O = T;
      var L = !0;
      try {
        L = P(!0, T);
      } finally {
        L ? R() : ((S = !1), (P = null));
      }
    } else S = !1;
  }
  var R;
  if (typeof p == "function")
    R = function () {
      p(H);
    };
  else if (typeof MessageChannel < "u") {
    var Y = new MessageChannel(),
      K = Y.port2;
    (Y.port1.onmessage = H),
      (R = function () {
        K.postMessage(null);
      });
  } else
    R = function () {
      E(H, 0);
    };
  function Q(T) {
    (P = T), S || ((S = !0), R());
  }
  function oe(T, L) {
    C = E(function () {
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
      g || x || ((g = !0), Q(k));
    }),
    (e.unstable_forceFrameRate = function (T) {
      0 > T || 125 < T
        ? console.error(
            "forceFrameRate takes a positive int between 0 and 125, forcing frame rates higher than 125 fps is not supported"
          )
        : (D = 0 < T ? Math.floor(1e3 / T) : 5);
    }),
    (e.unstable_getCurrentPriorityLevel = function () {
      return d;
    }),
    (e.unstable_getFirstCallbackNode = function () {
      return n(u);
    }),
    (e.unstable_next = function (T) {
      switch (d) {
        case 1:
        case 2:
        case 3:
          var L = 3;
          break;
        default:
          L = d;
      }
      var z = d;
      d = L;
      try {
        return T();
      } finally {
        d = z;
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
      var z = d;
      d = T;
      try {
        return L();
      } finally {
        d = z;
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
          id: c++,
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
              (v ? (m(C), (C = -1)) : (v = !0), oe(w, z - F)))
          : ((T.sortIndex = X), t(u, T), g || x || ((g = !0), Q(k))),
        T
      );
    }),
    (e.unstable_shouldYield = N),
    (e.unstable_wrapCallback = function (T) {
      var L = d;
      return function () {
        var z = d;
        d = L;
        try {
          return T.apply(this, arguments);
        } finally {
          d = z;
        }
      };
    });
})(sd);
ld.exports = sd;
var C0 = ld.exports;
/**
 * @license React
 * react-dom.production.min.js
 *
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */ var M0 = _,
  He = C0;
function M(e) {
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
var ud = new Set(),
  ei = {};
function jn(e, t) {
  or(e, t), or(e + "Capture", t);
}
function or(e, t) {
  for (ei[e] = t, e = 0; e < t.length; e++) ud.add(t[e]);
}
var jt = !(
    typeof window > "u" ||
    typeof window.document > "u" ||
    typeof window.document.createElement > "u"
  ),
  cs = Object.prototype.hasOwnProperty,
  j0 =
    /^[:A-Z_a-z\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD][:A-Z_a-z\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD\-.0-9\u00B7\u0300-\u036F\u203F-\u2040]*$/,
  Fa = {},
  Ia = {};
function T0(e) {
  return cs.call(Ia, e)
    ? !0
    : cs.call(Fa, e)
    ? !1
    : j0.test(e)
    ? (Ia[e] = !0)
    : ((Fa[e] = !0), !1);
}
function _0(e, t, n, r) {
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
function L0(e, t, n, r) {
  if (t === null || typeof t > "u" || _0(e, t, n, r)) return !0;
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
function je(e, t, n, r, i, o, l) {
  (this.acceptsBooleans = t === 2 || t === 3 || t === 4),
    (this.attributeName = r),
    (this.attributeNamespace = i),
    (this.mustUseProperty = n),
    (this.propertyName = e),
    (this.type = t),
    (this.sanitizeURL = o),
    (this.removeEmptyString = l);
}
var xe = {};
"children dangerouslySetInnerHTML defaultValue defaultChecked innerHTML suppressContentEditableWarning suppressHydrationWarning style"
  .split(" ")
  .forEach(function (e) {
    xe[e] = new je(e, 0, !1, e, null, !1, !1);
  });
[
  ["acceptCharset", "accept-charset"],
  ["className", "class"],
  ["htmlFor", "for"],
  ["httpEquiv", "http-equiv"],
].forEach(function (e) {
  var t = e[0];
  xe[t] = new je(t, 1, !1, e[1], null, !1, !1);
});
["contentEditable", "draggable", "spellCheck", "value"].forEach(function (e) {
  xe[e] = new je(e, 2, !1, e.toLowerCase(), null, !1, !1);
});
[
  "autoReverse",
  "externalResourcesRequired",
  "focusable",
  "preserveAlpha",
].forEach(function (e) {
  xe[e] = new je(e, 2, !1, e, null, !1, !1);
});
"allowFullScreen async autoFocus autoPlay controls default defer disabled disablePictureInPicture disableRemotePlayback formNoValidate hidden loop noModule noValidate open playsInline readOnly required reversed scoped seamless itemScope"
  .split(" ")
  .forEach(function (e) {
    xe[e] = new je(e, 3, !1, e.toLowerCase(), null, !1, !1);
  });
["checked", "multiple", "muted", "selected"].forEach(function (e) {
  xe[e] = new je(e, 3, !0, e, null, !1, !1);
});
["capture", "download"].forEach(function (e) {
  xe[e] = new je(e, 4, !1, e, null, !1, !1);
});
["cols", "rows", "size", "span"].forEach(function (e) {
  xe[e] = new je(e, 6, !1, e, null, !1, !1);
});
["rowSpan", "start"].forEach(function (e) {
  xe[e] = new je(e, 5, !1, e.toLowerCase(), null, !1, !1);
});
var ku = /[\-:]([a-z])/g;
function Su(e) {
  return e[1].toUpperCase();
}
"accent-height alignment-baseline arabic-form baseline-shift cap-height clip-path clip-rule color-interpolation color-interpolation-filters color-profile color-rendering dominant-baseline enable-background fill-opacity fill-rule flood-color flood-opacity font-family font-size font-size-adjust font-stretch font-style font-variant font-weight glyph-name glyph-orientation-horizontal glyph-orientation-vertical horiz-adv-x horiz-origin-x image-rendering letter-spacing lighting-color marker-end marker-mid marker-start overline-position overline-thickness paint-order panose-1 pointer-events rendering-intent shape-rendering stop-color stop-opacity strikethrough-position strikethrough-thickness stroke-dasharray stroke-dashoffset stroke-linecap stroke-linejoin stroke-miterlimit stroke-opacity stroke-width text-anchor text-decoration text-rendering underline-position underline-thickness unicode-bidi unicode-range units-per-em v-alphabetic v-hanging v-ideographic v-mathematical vector-effect vert-adv-y vert-origin-x vert-origin-y word-spacing writing-mode xmlns:xlink x-height"
  .split(" ")
  .forEach(function (e) {
    var t = e.replace(ku, Su);
    xe[t] = new je(t, 1, !1, e, null, !1, !1);
  });
"xlink:actuate xlink:arcrole xlink:role xlink:show xlink:title xlink:type"
  .split(" ")
  .forEach(function (e) {
    var t = e.replace(ku, Su);
    xe[t] = new je(t, 1, !1, e, "http://www.w3.org/1999/xlink", !1, !1);
  });
["xml:base", "xml:lang", "xml:space"].forEach(function (e) {
  var t = e.replace(ku, Su);
  xe[t] = new je(t, 1, !1, e, "http://www.w3.org/XML/1998/namespace", !1, !1);
});
["tabIndex", "crossOrigin"].forEach(function (e) {
  xe[e] = new je(e, 1, !1, e.toLowerCase(), null, !1, !1);
});
xe.xlinkHref = new je(
  "xlinkHref",
  1,
  !1,
  "xlink:href",
  "http://www.w3.org/1999/xlink",
  !0,
  !1
);
["src", "href", "action", "formAction"].forEach(function (e) {
  xe[e] = new je(e, 1, !1, e.toLowerCase(), null, !0, !0);
});
function Eu(e, t, n, r) {
  var i = xe.hasOwnProperty(t) ? xe[t] : null;
  (i !== null
    ? i.type !== 0
    : r ||
      !(2 < t.length) ||
      (t[0] !== "o" && t[0] !== "O") ||
      (t[1] !== "n" && t[1] !== "N")) &&
    (L0(t, n, i, r) && (n = null),
    r || i === null
      ? T0(t) && (n === null ? e.removeAttribute(t) : e.setAttribute(t, "" + n))
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
var At = M0.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED,
  zi = Symbol.for("react.element"),
  Rn = Symbol.for("react.portal"),
  Fn = Symbol.for("react.fragment"),
  Pu = Symbol.for("react.strict_mode"),
  fs = Symbol.for("react.profiler"),
  ad = Symbol.for("react.provider"),
  cd = Symbol.for("react.context"),
  Cu = Symbol.for("react.forward_ref"),
  ds = Symbol.for("react.suspense"),
  hs = Symbol.for("react.suspense_list"),
  Mu = Symbol.for("react.memo"),
  zt = Symbol.for("react.lazy"),
  fd = Symbol.for("react.offscreen"),
  Ua = Symbol.iterator;
function Er(e) {
  return e === null || typeof e != "object"
    ? null
    : ((e = (Ua && e[Ua]) || e["@@iterator"]),
      typeof e == "function" ? e : null);
}
var re = Object.assign,
  Sl;
function Or(e) {
  if (Sl === void 0)
    try {
      throw Error();
    } catch (n) {
      var t = n.stack.trim().match(/\n( *(at )?)/);
      Sl = (t && t[1]) || "";
    }
  return (
    `
` +
    Sl +
    e
  );
}
var El = !1;
function Pl(e, t) {
  if (!e || El) return "";
  El = !0;
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
    (El = !1), (Error.prepareStackTrace = n);
  }
  return (e = e ? e.displayName || e.name : "") ? Or(e) : "";
}
function N0(e) {
  switch (e.tag) {
    case 5:
      return Or(e.type);
    case 16:
      return Or("Lazy");
    case 13:
      return Or("Suspense");
    case 19:
      return Or("SuspenseList");
    case 0:
    case 2:
    case 15:
      return (e = Pl(e.type, !1)), e;
    case 11:
      return (e = Pl(e.type.render, !1)), e;
    case 1:
      return (e = Pl(e.type, !0)), e;
    default:
      return "";
  }
}
function ps(e) {
  if (e == null) return null;
  if (typeof e == "function") return e.displayName || e.name || null;
  if (typeof e == "string") return e;
  switch (e) {
    case Fn:
      return "Fragment";
    case Rn:
      return "Portal";
    case fs:
      return "Profiler";
    case Pu:
      return "StrictMode";
    case ds:
      return "Suspense";
    case hs:
      return "SuspenseList";
  }
  if (typeof e == "object")
    switch (e.$$typeof) {
      case cd:
        return (e.displayName || "Context") + ".Consumer";
      case ad:
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
          (t = e.displayName || null), t !== null ? t : ps(e.type) || "Memo"
        );
      case zt:
        (t = e._payload), (e = e._init);
        try {
          return ps(e(t));
        } catch {}
    }
  return null;
}
function $0(e) {
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
      return ps(t);
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
function dd(e) {
  var t = e.type;
  return (
    (e = e.nodeName) &&
    e.toLowerCase() === "input" &&
    (t === "checkbox" || t === "radio")
  );
}
function A0(e) {
  var t = dd(e) ? "checked" : "value",
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
function Oi(e) {
  e._valueTracker || (e._valueTracker = A0(e));
}
function hd(e) {
  if (!e) return !1;
  var t = e._valueTracker;
  if (!t) return !0;
  var n = t.getValue(),
    r = "";
  return (
    e && (r = dd(e) ? (e.checked ? "true" : "false") : e.value),
    (e = r),
    e !== n ? (t.setValue(e), !0) : !1
  );
}
function mo(e) {
  if (((e = e || (typeof document < "u" ? document : void 0)), typeof e > "u"))
    return null;
  try {
    return e.activeElement || e.body;
  } catch {
    return e.body;
  }
}
function ms(e, t) {
  var n = t.checked;
  return re({}, t, {
    defaultChecked: void 0,
    defaultValue: void 0,
    value: void 0,
    checked: n ?? e._wrapperState.initialChecked,
  });
}
function Ha(e, t) {
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
function pd(e, t) {
  (t = t.checked), t != null && Eu(e, "checked", t, !1);
}
function ys(e, t) {
  pd(e, t);
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
    ? gs(e, t.type, n)
    : t.hasOwnProperty("defaultValue") && gs(e, t.type, Zt(t.defaultValue)),
    t.checked == null &&
      t.defaultChecked != null &&
      (e.defaultChecked = !!t.defaultChecked);
}
function Wa(e, t, n) {
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
function gs(e, t, n) {
  (t !== "number" || mo(e.ownerDocument) !== e) &&
    (n == null
      ? (e.defaultValue = "" + e._wrapperState.initialValue)
      : e.defaultValue !== "" + n && (e.defaultValue = "" + n));
}
var Rr = Array.isArray;
function Jn(e, t, n, r) {
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
function xs(e, t) {
  if (t.dangerouslySetInnerHTML != null) throw Error(M(91));
  return re({}, t, {
    value: void 0,
    defaultValue: void 0,
    children: "" + e._wrapperState.initialValue,
  });
}
function Va(e, t) {
  var n = t.value;
  if (n == null) {
    if (((n = t.children), (t = t.defaultValue), n != null)) {
      if (t != null) throw Error(M(92));
      if (Rr(n)) {
        if (1 < n.length) throw Error(M(93));
        n = n[0];
      }
      t = n;
    }
    t == null && (t = ""), (n = t);
  }
  e._wrapperState = { initialValue: Zt(n) };
}
function md(e, t) {
  var n = Zt(t.value),
    r = Zt(t.defaultValue);
  n != null &&
    ((n = "" + n),
    n !== e.value && (e.value = n),
    t.defaultValue == null && e.defaultValue !== n && (e.defaultValue = n)),
    r != null && (e.defaultValue = "" + r);
}
function Ba(e) {
  var t = e.textContent;
  t === e._wrapperState.initialValue && t !== "" && t !== null && (e.value = t);
}
function yd(e) {
  switch (e) {
    case "svg":
      return "http://www.w3.org/2000/svg";
    case "math":
      return "http://www.w3.org/1998/Math/MathML";
    default:
      return "http://www.w3.org/1999/xhtml";
  }
}
function vs(e, t) {
  return e == null || e === "http://www.w3.org/1999/xhtml"
    ? yd(t)
    : e === "http://www.w3.org/2000/svg" && t === "foreignObject"
    ? "http://www.w3.org/1999/xhtml"
    : e;
}
var Ri,
  gd = (function (e) {
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
        Ri = Ri || document.createElement("div"),
          Ri.innerHTML = "<svg>" + t.valueOf().toString() + "</svg>",
          t = Ri.firstChild;
        e.firstChild;

      )
        e.removeChild(e.firstChild);
      for (; t.firstChild; ) e.appendChild(t.firstChild);
    }
  });
function ti(e, t) {
  if (t) {
    var n = e.firstChild;
    if (n && n === e.lastChild && n.nodeType === 3) {
      n.nodeValue = t;
      return;
    }
  }
  e.textContent = t;
}
var Br = {
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
  D0 = ["Webkit", "ms", "Moz", "O"];
Object.keys(Br).forEach(function (e) {
  D0.forEach(function (t) {
    (t = t + e.charAt(0).toUpperCase() + e.substring(1)), (Br[t] = Br[e]);
  });
});
function xd(e, t, n) {
  return t == null || typeof t == "boolean" || t === ""
    ? ""
    : n || typeof t != "number" || t === 0 || (Br.hasOwnProperty(e) && Br[e])
    ? ("" + t).trim()
    : t + "px";
}
function vd(e, t) {
  e = e.style;
  for (var n in t)
    if (t.hasOwnProperty(n)) {
      var r = n.indexOf("--") === 0,
        i = xd(n, t[n], r);
      n === "float" && (n = "cssFloat"), r ? e.setProperty(n, i) : (e[n] = i);
    }
}
var z0 = re(
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
function ws(e, t) {
  if (t) {
    if (z0[e] && (t.children != null || t.dangerouslySetInnerHTML != null))
      throw Error(M(137, e));
    if (t.dangerouslySetInnerHTML != null) {
      if (t.children != null) throw Error(M(60));
      if (
        typeof t.dangerouslySetInnerHTML != "object" ||
        !("__html" in t.dangerouslySetInnerHTML)
      )
        throw Error(M(61));
    }
    if (t.style != null && typeof t.style != "object") throw Error(M(62));
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
var Ss = null;
function ju(e) {
  return (
    (e = e.target || e.srcElement || window),
    e.correspondingUseElement && (e = e.correspondingUseElement),
    e.nodeType === 3 ? e.parentNode : e
  );
}
var Es = null,
  bn = null,
  er = null;
function Ya(e) {
  if ((e = ji(e))) {
    if (typeof Es != "function") throw Error(M(280));
    var t = e.stateNode;
    t && ((t = rl(t)), Es(e.stateNode, e.type, t));
  }
}
function wd(e) {
  bn ? (er ? er.push(e) : (er = [e])) : (bn = e);
}
function kd() {
  if (bn) {
    var e = bn,
      t = er;
    if (((er = bn = null), Ya(e), t)) for (e = 0; e < t.length; e++) Ya(t[e]);
  }
}
function Sd(e, t) {
  return e(t);
}
function Ed() {}
var Cl = !1;
function Pd(e, t, n) {
  if (Cl) return e(t, n);
  Cl = !0;
  try {
    return Sd(e, t, n);
  } finally {
    (Cl = !1), (bn !== null || er !== null) && (Ed(), kd());
  }
}
function ni(e, t) {
  var n = e.stateNode;
  if (n === null) return null;
  var r = rl(n);
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
  if (n && typeof n != "function") throw Error(M(231, t, typeof n));
  return n;
}
var Ps = !1;
if (jt)
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
function O0(e, t, n, r, i, o, l, s, u) {
  var a = Array.prototype.slice.call(arguments, 3);
  try {
    t.apply(n, a);
  } catch (c) {
    this.onError(c);
  }
}
var Yr = !1,
  yo = null,
  go = !1,
  Cs = null,
  R0 = {
    onError: function (e) {
      (Yr = !0), (yo = e);
    },
  };
function F0(e, t, n, r, i, o, l, s, u) {
  (Yr = !1), (yo = null), O0.apply(R0, arguments);
}
function I0(e, t, n, r, i, o, l, s, u) {
  if ((F0.apply(this, arguments), Yr)) {
    if (Yr) {
      var a = yo;
      (Yr = !1), (yo = null);
    } else throw Error(M(198));
    go || ((go = !0), (Cs = a));
  }
}
function Tn(e) {
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
  if (Tn(e) !== e) throw Error(M(188));
}
function U0(e) {
  var t = e.alternate;
  if (!t) {
    if (((t = Tn(e)), t === null)) throw Error(M(188));
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
      throw Error(M(188));
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
        if (!l) throw Error(M(189));
      }
    }
    if (n.alternate !== r) throw Error(M(190));
  }
  if (n.tag !== 3) throw Error(M(188));
  return n.stateNode.current === n ? e : t;
}
function Md(e) {
  return (e = U0(e)), e !== null ? jd(e) : null;
}
function jd(e) {
  if (e.tag === 5 || e.tag === 6) return e;
  for (e = e.child; e !== null; ) {
    var t = jd(e);
    if (t !== null) return t;
    e = e.sibling;
  }
  return null;
}
var Td = He.unstable_scheduleCallback,
  Qa = He.unstable_cancelCallback,
  H0 = He.unstable_shouldYield,
  W0 = He.unstable_requestPaint,
  le = He.unstable_now,
  V0 = He.unstable_getCurrentPriorityLevel,
  Tu = He.unstable_ImmediatePriority,
  _d = He.unstable_UserBlockingPriority,
  xo = He.unstable_NormalPriority,
  B0 = He.unstable_LowPriority,
  Ld = He.unstable_IdlePriority,
  bo = null,
  mt = null;
function Y0(e) {
  if (mt && typeof mt.onCommitFiberRoot == "function")
    try {
      mt.onCommitFiberRoot(bo, e, void 0, (e.current.flags & 128) === 128);
    } catch {}
}
var ut = Math.clz32 ? Math.clz32 : X0,
  G0 = Math.log,
  Q0 = Math.LN2;
function X0(e) {
  return (e >>>= 0), e === 0 ? 32 : (31 - ((G0(e) / Q0) | 0)) | 0;
}
var Fi = 64,
  Ii = 4194304;
function Fr(e) {
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
function vo(e, t) {
  var n = e.pendingLanes;
  if (n === 0) return 0;
  var r = 0,
    i = e.suspendedLanes,
    o = e.pingedLanes,
    l = n & 268435455;
  if (l !== 0) {
    var s = l & ~i;
    s !== 0 ? (r = Fr(s)) : ((o &= l), o !== 0 && (r = Fr(o)));
  } else (l = n & ~i), l !== 0 ? (r = Fr(l)) : o !== 0 && (r = Fr(o));
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
      (n = 31 - ut(t)), (i = 1 << n), (r |= e[n]), (t &= ~i);
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
    var l = 31 - ut(o),
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
function Nd() {
  var e = Fi;
  return (Fi <<= 1), !(Fi & 4194240) && (Fi = 64), e;
}
function Ml(e) {
  for (var t = [], n = 0; 31 > n; n++) t.push(e);
  return t;
}
function Ci(e, t, n) {
  (e.pendingLanes |= t),
    t !== 536870912 && ((e.suspendedLanes = 0), (e.pingedLanes = 0)),
    (e = e.eventTimes),
    (t = 31 - ut(t)),
    (e[t] = n);
}
function q0(e, t) {
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
    var i = 31 - ut(n),
      o = 1 << i;
    (t[i] = 0), (r[i] = -1), (e[i] = -1), (n &= ~o);
  }
}
function _u(e, t) {
  var n = (e.entangledLanes |= t);
  for (e = e.entanglements; n; ) {
    var r = 31 - ut(n),
      i = 1 << r;
    (i & t) | (e[r] & t) && (e[r] |= t), (n &= ~i);
  }
}
var G = 0;
function $d(e) {
  return (e &= -e), 1 < e ? (4 < e ? (e & 268435455 ? 16 : 536870912) : 4) : 1;
}
var Ad,
  Lu,
  Dd,
  zd,
  Od,
  js = !1,
  Ui = [],
  Wt = null,
  Vt = null,
  Bt = null,
  ri = new Map(),
  ii = new Map(),
  Ft = [],
  J0 =
    "mousedown mouseup touchcancel touchend touchstart auxclick dblclick pointercancel pointerdown pointerup dragend dragstart drop compositionend compositionstart keydown keypress keyup input textInput copy cut paste click change contextmenu reset submit".split(
      " "
    );
function Xa(e, t) {
  switch (e) {
    case "focusin":
    case "focusout":
      Wt = null;
      break;
    case "dragenter":
    case "dragleave":
      Vt = null;
      break;
    case "mouseover":
    case "mouseout":
      Bt = null;
      break;
    case "pointerover":
    case "pointerout":
      ri.delete(t.pointerId);
      break;
    case "gotpointercapture":
    case "lostpointercapture":
      ii.delete(t.pointerId);
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
      t !== null && ((t = ji(t)), t !== null && Lu(t)),
      e)
    : ((e.eventSystemFlags |= r),
      (t = e.targetContainers),
      i !== null && t.indexOf(i) === -1 && t.push(i),
      e);
}
function b0(e, t, n, r, i) {
  switch (t) {
    case "focusin":
      return (Wt = Cr(Wt, e, t, n, r, i)), !0;
    case "dragenter":
      return (Vt = Cr(Vt, e, t, n, r, i)), !0;
    case "mouseover":
      return (Bt = Cr(Bt, e, t, n, r, i)), !0;
    case "pointerover":
      var o = i.pointerId;
      return ri.set(o, Cr(ri.get(o) || null, e, t, n, r, i)), !0;
    case "gotpointercapture":
      return (
        (o = i.pointerId), ii.set(o, Cr(ii.get(o) || null, e, t, n, r, i)), !0
      );
  }
  return !1;
}
function Rd(e) {
  var t = an(e.target);
  if (t !== null) {
    var n = Tn(t);
    if (n !== null) {
      if (((t = n.tag), t === 13)) {
        if (((t = Cd(n)), t !== null)) {
          (e.blockedOn = t),
            Od(e.priority, function () {
              Dd(n);
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
function ro(e) {
  if (e.blockedOn !== null) return !1;
  for (var t = e.targetContainers; 0 < t.length; ) {
    var n = Ts(e.domEventName, e.eventSystemFlags, t[0], e.nativeEvent);
    if (n === null) {
      n = e.nativeEvent;
      var r = new n.constructor(n.type, n);
      (Ss = r), n.target.dispatchEvent(r), (Ss = null);
    } else return (t = ji(n)), t !== null && Lu(t), (e.blockedOn = n), !1;
    t.shift();
  }
  return !0;
}
function Ka(e, t, n) {
  ro(e) && n.delete(t);
}
function em() {
  (js = !1),
    Wt !== null && ro(Wt) && (Wt = null),
    Vt !== null && ro(Vt) && (Vt = null),
    Bt !== null && ro(Bt) && (Bt = null),
    ri.forEach(Ka),
    ii.forEach(Ka);
}
function Mr(e, t) {
  e.blockedOn === t &&
    ((e.blockedOn = null),
    js ||
      ((js = !0),
      He.unstable_scheduleCallback(He.unstable_NormalPriority, em)));
}
function oi(e) {
  function t(i) {
    return Mr(i, e);
  }
  if (0 < Ui.length) {
    Mr(Ui[0], e);
    for (var n = 1; n < Ui.length; n++) {
      var r = Ui[n];
      r.blockedOn === e && (r.blockedOn = null);
    }
  }
  for (
    Wt !== null && Mr(Wt, e),
      Vt !== null && Mr(Vt, e),
      Bt !== null && Mr(Bt, e),
      ri.forEach(t),
      ii.forEach(t),
      n = 0;
    n < Ft.length;
    n++
  )
    (r = Ft[n]), r.blockedOn === e && (r.blockedOn = null);
  for (; 0 < Ft.length && ((n = Ft[0]), n.blockedOn === null); )
    Rd(n), n.blockedOn === null && Ft.shift();
}
var tr = At.ReactCurrentBatchConfig,
  wo = !0;
function tm(e, t, n, r) {
  var i = G,
    o = tr.transition;
  tr.transition = null;
  try {
    (G = 1), Nu(e, t, n, r);
  } finally {
    (G = i), (tr.transition = o);
  }
}
function nm(e, t, n, r) {
  var i = G,
    o = tr.transition;
  tr.transition = null;
  try {
    (G = 4), Nu(e, t, n, r);
  } finally {
    (G = i), (tr.transition = o);
  }
}
function Nu(e, t, n, r) {
  if (wo) {
    var i = Ts(e, t, n, r);
    if (i === null) Ol(e, t, r, ko, n), Xa(e, r);
    else if (b0(i, e, t, n, r)) r.stopPropagation();
    else if ((Xa(e, r), t & 4 && -1 < J0.indexOf(e))) {
      for (; i !== null; ) {
        var o = ji(i);
        if (
          (o !== null && Ad(o),
          (o = Ts(e, t, n, r)),
          o === null && Ol(e, t, r, ko, n),
          o === i)
        )
          break;
        i = o;
      }
      i !== null && r.stopPropagation();
    } else Ol(e, t, r, null, n);
  }
}
var ko = null;
function Ts(e, t, n, r) {
  if (((ko = null), (e = ju(r)), (e = an(e)), e !== null))
    if (((t = Tn(e)), t === null)) e = null;
    else if (((n = t.tag), n === 13)) {
      if (((e = Cd(t)), e !== null)) return e;
      e = null;
    } else if (n === 3) {
      if (t.stateNode.current.memoizedState.isDehydrated)
        return t.tag === 3 ? t.stateNode.containerInfo : null;
      e = null;
    } else t !== e && (e = null);
  return (ko = e), null;
}
function Fd(e) {
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
      switch (V0()) {
        case Tu:
          return 1;
        case _d:
          return 4;
        case xo:
        case B0:
          return 16;
        case Ld:
          return 536870912;
        default:
          return 16;
      }
    default:
      return 16;
  }
}
var Ut = null,
  $u = null,
  io = null;
function Id() {
  if (io) return io;
  var e,
    t = $u,
    n = t.length,
    r,
    i = "value" in Ut ? Ut.value : Ut.textContent,
    o = i.length;
  for (e = 0; e < n && t[e] === i[e]; e++);
  var l = n - e;
  for (r = 1; r <= l && t[n - r] === i[o - r]; r++);
  return (io = i.slice(e, 1 < r ? 1 - r : void 0));
}
function oo(e) {
  var t = e.keyCode;
  return (
    "charCode" in e
      ? ((e = e.charCode), e === 0 && t === 13 && (e = 13))
      : (e = t),
    e === 10 && (e = 13),
    32 <= e || e === 13 ? e : 0
  );
}
function Hi() {
  return !0;
}
function Za() {
  return !1;
}
function Ye(e) {
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
        ? Hi
        : Za),
      (this.isPropagationStopped = Za),
      this
    );
  }
  return (
    re(t.prototype, {
      preventDefault: function () {
        this.defaultPrevented = !0;
        var n = this.nativeEvent;
        n &&
          (n.preventDefault
            ? n.preventDefault()
            : typeof n.returnValue != "unknown" && (n.returnValue = !1),
          (this.isDefaultPrevented = Hi));
      },
      stopPropagation: function () {
        var n = this.nativeEvent;
        n &&
          (n.stopPropagation
            ? n.stopPropagation()
            : typeof n.cancelBubble != "unknown" && (n.cancelBubble = !0),
          (this.isPropagationStopped = Hi));
      },
      persist: function () {},
      isPersistent: Hi,
    }),
    t
  );
}
var gr = {
    eventPhase: 0,
    bubbles: 0,
    cancelable: 0,
    timeStamp: function (e) {
      return e.timeStamp || Date.now();
    },
    defaultPrevented: 0,
    isTrusted: 0,
  },
  Au = Ye(gr),
  Mi = re({}, gr, { view: 0, detail: 0 }),
  rm = Ye(Mi),
  jl,
  Tl,
  jr,
  el = re({}, Mi, {
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
    getModifierState: Du,
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
        : (e !== jr &&
            (jr && e.type === "mousemove"
              ? ((jl = e.screenX - jr.screenX), (Tl = e.screenY - jr.screenY))
              : (Tl = jl = 0),
            (jr = e)),
          jl);
    },
    movementY: function (e) {
      return "movementY" in e ? e.movementY : Tl;
    },
  }),
  qa = Ye(el),
  im = re({}, el, { dataTransfer: 0 }),
  om = Ye(im),
  lm = re({}, Mi, { relatedTarget: 0 }),
  _l = Ye(lm),
  sm = re({}, gr, { animationName: 0, elapsedTime: 0, pseudoElement: 0 }),
  um = Ye(sm),
  am = re({}, gr, {
    clipboardData: function (e) {
      return "clipboardData" in e ? e.clipboardData : window.clipboardData;
    },
  }),
  cm = Ye(am),
  fm = re({}, gr, { data: 0 }),
  Ja = Ye(fm),
  dm = {
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
  hm = {
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
  pm = {
    Alt: "altKey",
    Control: "ctrlKey",
    Meta: "metaKey",
    Shift: "shiftKey",
  };
function mm(e) {
  var t = this.nativeEvent;
  return t.getModifierState ? t.getModifierState(e) : (e = pm[e]) ? !!t[e] : !1;
}
function Du() {
  return mm;
}
var ym = re({}, Mi, {
    key: function (e) {
      if (e.key) {
        var t = dm[e.key] || e.key;
        if (t !== "Unidentified") return t;
      }
      return e.type === "keypress"
        ? ((e = oo(e)), e === 13 ? "Enter" : String.fromCharCode(e))
        : e.type === "keydown" || e.type === "keyup"
        ? hm[e.keyCode] || "Unidentified"
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
    getModifierState: Du,
    charCode: function (e) {
      return e.type === "keypress" ? oo(e) : 0;
    },
    keyCode: function (e) {
      return e.type === "keydown" || e.type === "keyup" ? e.keyCode : 0;
    },
    which: function (e) {
      return e.type === "keypress"
        ? oo(e)
        : e.type === "keydown" || e.type === "keyup"
        ? e.keyCode
        : 0;
    },
  }),
  gm = Ye(ym),
  xm = re({}, el, {
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
  ba = Ye(xm),
  vm = re({}, Mi, {
    touches: 0,
    targetTouches: 0,
    changedTouches: 0,
    altKey: 0,
    metaKey: 0,
    ctrlKey: 0,
    shiftKey: 0,
    getModifierState: Du,
  }),
  wm = Ye(vm),
  km = re({}, gr, { propertyName: 0, elapsedTime: 0, pseudoElement: 0 }),
  Sm = Ye(km),
  Em = re({}, el, {
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
  Pm = Ye(Em),
  Cm = [9, 13, 27, 32],
  zu = jt && "CompositionEvent" in window,
  Gr = null;
jt && "documentMode" in document && (Gr = document.documentMode);
var Mm = jt && "TextEvent" in window && !Gr,
  Ud = jt && (!zu || (Gr && 8 < Gr && 11 >= Gr)),
  ec = " ",
  tc = !1;
function Hd(e, t) {
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
function Wd(e) {
  return (e = e.detail), typeof e == "object" && "data" in e ? e.data : null;
}
var In = !1;
function jm(e, t) {
  switch (e) {
    case "compositionend":
      return Wd(t);
    case "keypress":
      return t.which !== 32 ? null : ((tc = !0), ec);
    case "textInput":
      return (e = t.data), e === ec && tc ? null : e;
    default:
      return null;
  }
}
function Tm(e, t) {
  if (In)
    return e === "compositionend" || (!zu && Hd(e, t))
      ? ((e = Id()), (io = $u = Ut = null), (In = !1), e)
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
      return Ud && t.locale !== "ko" ? null : t.data;
    default:
      return null;
  }
}
var _m = {
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
function nc(e) {
  var t = e && e.nodeName && e.nodeName.toLowerCase();
  return t === "input" ? !!_m[e.type] : t === "textarea";
}
function Vd(e, t, n, r) {
  wd(r),
    (t = So(t, "onChange")),
    0 < t.length &&
      ((n = new Au("onChange", "change", null, n, r)),
      e.push({ event: n, listeners: t }));
}
var Qr = null,
  li = null;
function Lm(e) {
  eh(e, 0);
}
function tl(e) {
  var t = Wn(e);
  if (hd(t)) return e;
}
function Nm(e, t) {
  if (e === "change") return t;
}
var Bd = !1;
if (jt) {
  var Ll;
  if (jt) {
    var Nl = "oninput" in document;
    if (!Nl) {
      var rc = document.createElement("div");
      rc.setAttribute("oninput", "return;"),
        (Nl = typeof rc.oninput == "function");
    }
    Ll = Nl;
  } else Ll = !1;
  Bd = Ll && (!document.documentMode || 9 < document.documentMode);
}
function ic() {
  Qr && (Qr.detachEvent("onpropertychange", Yd), (li = Qr = null));
}
function Yd(e) {
  if (e.propertyName === "value" && tl(li)) {
    var t = [];
    Vd(t, li, e, ju(e)), Pd(Lm, t);
  }
}
function $m(e, t, n) {
  e === "focusin"
    ? (ic(), (Qr = t), (li = n), Qr.attachEvent("onpropertychange", Yd))
    : e === "focusout" && ic();
}
function Am(e) {
  if (e === "selectionchange" || e === "keyup" || e === "keydown")
    return tl(li);
}
function Dm(e, t) {
  if (e === "click") return tl(t);
}
function zm(e, t) {
  if (e === "input" || e === "change") return tl(t);
}
function Om(e, t) {
  return (e === t && (e !== 0 || 1 / e === 1 / t)) || (e !== e && t !== t);
}
var ct = typeof Object.is == "function" ? Object.is : Om;
function si(e, t) {
  if (ct(e, t)) return !0;
  if (typeof e != "object" || e === null || typeof t != "object" || t === null)
    return !1;
  var n = Object.keys(e),
    r = Object.keys(t);
  if (n.length !== r.length) return !1;
  for (r = 0; r < n.length; r++) {
    var i = n[r];
    if (!cs.call(t, i) || !ct(e[i], t[i])) return !1;
  }
  return !0;
}
function oc(e) {
  for (; e && e.firstChild; ) e = e.firstChild;
  return e;
}
function lc(e, t) {
  var n = oc(e);
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
    n = oc(n);
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
function Qd() {
  for (var e = window, t = mo(); t instanceof e.HTMLIFrameElement; ) {
    try {
      var n = typeof t.contentWindow.location.href == "string";
    } catch {
      n = !1;
    }
    if (n) e = t.contentWindow;
    else break;
    t = mo(e.document);
  }
  return t;
}
function Ou(e) {
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
function Rm(e) {
  var t = Qd(),
    n = e.focusedElem,
    r = e.selectionRange;
  if (
    t !== n &&
    n &&
    n.ownerDocument &&
    Gd(n.ownerDocument.documentElement, n)
  ) {
    if (r !== null && Ou(n)) {
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
          (i = lc(n, o));
        var l = lc(n, r);
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
var Fm = jt && "documentMode" in document && 11 >= document.documentMode,
  Un = null,
  _s = null,
  Xr = null,
  Ls = !1;
function sc(e, t, n) {
  var r = n.window === n ? n.document : n.nodeType === 9 ? n : n.ownerDocument;
  Ls ||
    Un == null ||
    Un !== mo(r) ||
    ((r = Un),
    "selectionStart" in r && Ou(r)
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
    (Xr && si(Xr, r)) ||
      ((Xr = r),
      (r = So(_s, "onSelect")),
      0 < r.length &&
        ((t = new Au("onSelect", "select", null, t, n)),
        e.push({ event: t, listeners: r }),
        (t.target = Un))));
}
function Wi(e, t) {
  var n = {};
  return (
    (n[e.toLowerCase()] = t.toLowerCase()),
    (n["Webkit" + e] = "webkit" + t),
    (n["Moz" + e] = "moz" + t),
    n
  );
}
var Hn = {
    animationend: Wi("Animation", "AnimationEnd"),
    animationiteration: Wi("Animation", "AnimationIteration"),
    animationstart: Wi("Animation", "AnimationStart"),
    transitionend: Wi("Transition", "TransitionEnd"),
  },
  $l = {},
  Xd = {};
jt &&
  ((Xd = document.createElement("div").style),
  "AnimationEvent" in window ||
    (delete Hn.animationend.animation,
    delete Hn.animationiteration.animation,
    delete Hn.animationstart.animation),
  "TransitionEvent" in window || delete Hn.transitionend.transition);
function nl(e) {
  if ($l[e]) return $l[e];
  if (!Hn[e]) return e;
  var t = Hn[e],
    n;
  for (n in t) if (t.hasOwnProperty(n) && n in Xd) return ($l[e] = t[n]);
  return e;
}
var Kd = nl("animationend"),
  Zd = nl("animationiteration"),
  qd = nl("animationstart"),
  Jd = nl("transitionend"),
  bd = new Map(),
  uc =
    "abort auxClick cancel canPlay canPlayThrough click close contextMenu copy cut drag dragEnd dragEnter dragExit dragLeave dragOver dragStart drop durationChange emptied encrypted ended error gotPointerCapture input invalid keyDown keyPress keyUp load loadedData loadedMetadata loadStart lostPointerCapture mouseDown mouseMove mouseOut mouseOver mouseUp paste pause play playing pointerCancel pointerDown pointerMove pointerOut pointerOver pointerUp progress rateChange reset resize seeked seeking stalled submit suspend timeUpdate touchCancel touchEnd touchStart volumeChange scroll toggle touchMove waiting wheel".split(
      " "
    );
function Jt(e, t) {
  bd.set(e, t), jn(t, [e]);
}
for (var Al = 0; Al < uc.length; Al++) {
  var Dl = uc[Al],
    Im = Dl.toLowerCase(),
    Um = Dl[0].toUpperCase() + Dl.slice(1);
  Jt(Im, "on" + Um);
}
Jt(Kd, "onAnimationEnd");
Jt(Zd, "onAnimationIteration");
Jt(qd, "onAnimationStart");
Jt("dblclick", "onDoubleClick");
Jt("focusin", "onFocus");
Jt("focusout", "onBlur");
Jt(Jd, "onTransitionEnd");
or("onMouseEnter", ["mouseout", "mouseover"]);
or("onMouseLeave", ["mouseout", "mouseover"]);
or("onPointerEnter", ["pointerout", "pointerover"]);
or("onPointerLeave", ["pointerout", "pointerover"]);
jn(
  "onChange",
  "change click focusin focusout input keydown keyup selectionchange".split(" ")
);
jn(
  "onSelect",
  "focusout contextmenu dragend focusin keydown keyup mousedown mouseup selectionchange".split(
    " "
  )
);
jn("onBeforeInput", ["compositionend", "keypress", "textInput", "paste"]);
jn(
  "onCompositionEnd",
  "compositionend focusout keydown keypress keyup mousedown".split(" ")
);
jn(
  "onCompositionStart",
  "compositionstart focusout keydown keypress keyup mousedown".split(" ")
);
jn(
  "onCompositionUpdate",
  "compositionupdate focusout keydown keypress keyup mousedown".split(" ")
);
var Ir =
    "abort canplay canplaythrough durationchange emptied encrypted ended error loadeddata loadedmetadata loadstart pause play playing progress ratechange resize seeked seeking stalled suspend timeupdate volumechange waiting".split(
      " "
    ),
  Hm = new Set("cancel close invalid load scroll toggle".split(" ").concat(Ir));
function ac(e, t, n) {
  var r = e.type || "unknown-event";
  (e.currentTarget = n), I0(r, t, void 0, e), (e.currentTarget = null);
}
function eh(e, t) {
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
          ac(i, s, a), (o = u);
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
          ac(i, s, a), (o = u);
        }
    }
  }
  if (go) throw ((e = Cs), (go = !1), (Cs = null), e);
}
function J(e, t) {
  var n = t[zs];
  n === void 0 && (n = t[zs] = new Set());
  var r = e + "__bubble";
  n.has(r) || (th(t, e, 2, !1), n.add(r));
}
function zl(e, t, n) {
  var r = 0;
  t && (r |= 4), th(n, e, r, t);
}
var Vi = "_reactListening" + Math.random().toString(36).slice(2);
function ui(e) {
  if (!e[Vi]) {
    (e[Vi] = !0),
      ud.forEach(function (n) {
        n !== "selectionchange" && (Hm.has(n) || zl(n, !1, e), zl(n, !0, e));
      });
    var t = e.nodeType === 9 ? e : e.ownerDocument;
    t === null || t[Vi] || ((t[Vi] = !0), zl("selectionchange", !1, t));
  }
}
function th(e, t, n, r) {
  switch (Fd(t)) {
    case 1:
      var i = tm;
      break;
    case 4:
      i = nm;
      break;
    default:
      i = Nu;
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
function Ol(e, t, n, r, i) {
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
          if (((l = an(s)), l === null)) return;
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
      c = ju(n),
      f = [];
    e: {
      var d = bd.get(e);
      if (d !== void 0) {
        var x = Au,
          g = e;
        switch (e) {
          case "keypress":
            if (oo(n) === 0) break e;
          case "keydown":
          case "keyup":
            x = gm;
            break;
          case "focusin":
            (g = "focus"), (x = _l);
            break;
          case "focusout":
            (g = "blur"), (x = _l);
            break;
          case "beforeblur":
          case "afterblur":
            x = _l;
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
            x = qa;
            break;
          case "drag":
          case "dragend":
          case "dragenter":
          case "dragexit":
          case "dragleave":
          case "dragover":
          case "dragstart":
          case "drop":
            x = om;
            break;
          case "touchcancel":
          case "touchend":
          case "touchmove":
          case "touchstart":
            x = wm;
            break;
          case Kd:
          case Zd:
          case qd:
            x = um;
            break;
          case Jd:
            x = Sm;
            break;
          case "scroll":
            x = rm;
            break;
          case "wheel":
            x = Pm;
            break;
          case "copy":
          case "cut":
          case "paste":
            x = cm;
            break;
          case "gotpointercapture":
          case "lostpointercapture":
          case "pointercancel":
          case "pointerdown":
          case "pointermove":
          case "pointerout":
          case "pointerover":
          case "pointerup":
            x = ba;
        }
        var v = (t & 4) !== 0,
          E = !v && e === "scroll",
          m = v ? (d !== null ? d + "Capture" : null) : d;
        v = [];
        for (var p = a, y; p !== null; ) {
          y = p;
          var w = y.stateNode;
          if (
            (y.tag === 5 &&
              w !== null &&
              ((y = w),
              m !== null && ((w = ni(p, m)), w != null && v.push(ai(p, w, y)))),
            E)
          )
            break;
          p = p.return;
        }
        0 < v.length &&
          ((d = new x(d, g, null, n, c)), f.push({ event: d, listeners: v }));
      }
    }
    if (!(t & 7)) {
      e: {
        if (
          ((d = e === "mouseover" || e === "pointerover"),
          (x = e === "mouseout" || e === "pointerout"),
          d &&
            n !== Ss &&
            (g = n.relatedTarget || n.fromElement) &&
            (an(g) || g[Tt]))
        )
          break e;
        if (
          (x || d) &&
          ((d =
            c.window === c
              ? c
              : (d = c.ownerDocument)
              ? d.defaultView || d.parentWindow
              : window),
          x
            ? ((g = n.relatedTarget || n.toElement),
              (x = a),
              (g = g ? an(g) : null),
              g !== null &&
                ((E = Tn(g)), g !== E || (g.tag !== 5 && g.tag !== 6)) &&
                (g = null))
            : ((x = null), (g = a)),
          x !== g)
        ) {
          if (
            ((v = qa),
            (w = "onMouseLeave"),
            (m = "onMouseEnter"),
            (p = "mouse"),
            (e === "pointerout" || e === "pointerover") &&
              ((v = ba),
              (w = "onPointerLeave"),
              (m = "onPointerEnter"),
              (p = "pointer")),
            (E = x == null ? d : Wn(x)),
            (y = g == null ? d : Wn(g)),
            (d = new v(w, p + "leave", x, n, c)),
            (d.target = E),
            (d.relatedTarget = y),
            (w = null),
            an(c) === a &&
              ((v = new v(m, p + "enter", g, n, c)),
              (v.target = y),
              (v.relatedTarget = E),
              (w = v)),
            (E = w),
            x && g)
          )
            t: {
              for (v = x, m = g, p = 0, y = v; y; y = $n(y)) p++;
              for (y = 0, w = m; w; w = $n(w)) y++;
              for (; 0 < p - y; ) (v = $n(v)), p--;
              for (; 0 < y - p; ) (m = $n(m)), y--;
              for (; p--; ) {
                if (v === m || (m !== null && v === m.alternate)) break t;
                (v = $n(v)), (m = $n(m));
              }
              v = null;
            }
          else v = null;
          x !== null && cc(f, d, x, v, !1),
            g !== null && E !== null && cc(f, E, g, v, !0);
        }
      }
      e: {
        if (
          ((d = a ? Wn(a) : window),
          (x = d.nodeName && d.nodeName.toLowerCase()),
          x === "select" || (x === "input" && d.type === "file"))
        )
          var k = Nm;
        else if (nc(d))
          if (Bd) k = zm;
          else {
            k = Am;
            var S = $m;
          }
        else
          (x = d.nodeName) &&
            x.toLowerCase() === "input" &&
            (d.type === "checkbox" || d.type === "radio") &&
            (k = Dm);
        if (k && (k = k(e, a))) {
          Vd(f, k, n, c);
          break e;
        }
        S && S(e, d, a),
          e === "focusout" &&
            (S = d._wrapperState) &&
            S.controlled &&
            d.type === "number" &&
            gs(d, "number", d.value);
      }
      switch (((S = a ? Wn(a) : window), e)) {
        case "focusin":
          (nc(S) || S.contentEditable === "true") &&
            ((Un = S), (_s = a), (Xr = null));
          break;
        case "focusout":
          Xr = _s = Un = null;
          break;
        case "mousedown":
          Ls = !0;
          break;
        case "contextmenu":
        case "mouseup":
        case "dragend":
          (Ls = !1), sc(f, n, c);
          break;
        case "selectionchange":
          if (Fm) break;
        case "keydown":
        case "keyup":
          sc(f, n, c);
      }
      var P;
      if (zu)
        e: {
          switch (e) {
            case "compositionstart":
              var C = "onCompositionStart";
              break e;
            case "compositionend":
              C = "onCompositionEnd";
              break e;
            case "compositionupdate":
              C = "onCompositionUpdate";
              break e;
          }
          C = void 0;
        }
      else
        In
          ? Hd(e, n) && (C = "onCompositionEnd")
          : e === "keydown" && n.keyCode === 229 && (C = "onCompositionStart");
      C &&
        (Ud &&
          n.locale !== "ko" &&
          (In || C !== "onCompositionStart"
            ? C === "onCompositionEnd" && In && (P = Id())
            : ((Ut = c),
              ($u = "value" in Ut ? Ut.value : Ut.textContent),
              (In = !0))),
        (S = So(a, C)),
        0 < S.length &&
          ((C = new Ja(C, e, null, n, c)),
          f.push({ event: C, listeners: S }),
          P ? (C.data = P) : ((P = Wd(n)), P !== null && (C.data = P)))),
        (P = Mm ? jm(e, n) : Tm(e, n)) &&
          ((a = So(a, "onBeforeInput")),
          0 < a.length &&
            ((c = new Ja("onBeforeInput", "beforeinput", null, n, c)),
            f.push({ event: c, listeners: a }),
            (c.data = P)));
    }
    eh(f, t);
  });
}
function ai(e, t, n) {
  return { instance: e, listener: t, currentTarget: n };
}
function So(e, t) {
  for (var n = t + "Capture", r = []; e !== null; ) {
    var i = e,
      o = i.stateNode;
    i.tag === 5 &&
      o !== null &&
      ((i = o),
      (o = ni(e, n)),
      o != null && r.unshift(ai(e, o, i)),
      (o = ni(e, t)),
      o != null && r.push(ai(e, o, i))),
      (e = e.return);
  }
  return r;
}
function $n(e) {
  if (e === null) return null;
  do e = e.return;
  while (e && e.tag !== 5);
  return e || null;
}
function cc(e, t, n, r, i) {
  for (var o = t._reactName, l = []; n !== null && n !== r; ) {
    var s = n,
      u = s.alternate,
      a = s.stateNode;
    if (u !== null && u === r) break;
    s.tag === 5 &&
      a !== null &&
      ((s = a),
      i
        ? ((u = ni(n, o)), u != null && l.unshift(ai(n, u, s)))
        : i || ((u = ni(n, o)), u != null && l.push(ai(n, u, s)))),
      (n = n.return);
  }
  l.length !== 0 && e.push({ event: t, listeners: l });
}
var Wm = /\r\n?/g,
  Vm = /\u0000|\uFFFD/g;
function fc(e) {
  return (typeof e == "string" ? e : "" + e)
    .replace(
      Wm,
      `
`
    )
    .replace(Vm, "");
}
function Bi(e, t, n) {
  if (((t = fc(t)), fc(e) !== t && n)) throw Error(M(425));
}
function Eo() {}
var Ns = null,
  $s = null;
function As(e, t) {
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
var Ds = typeof setTimeout == "function" ? setTimeout : void 0,
  Bm = typeof clearTimeout == "function" ? clearTimeout : void 0,
  dc = typeof Promise == "function" ? Promise : void 0,
  Ym =
    typeof queueMicrotask == "function"
      ? queueMicrotask
      : typeof dc < "u"
      ? function (e) {
          return dc.resolve(null).then(e).catch(Gm);
        }
      : Ds;
function Gm(e) {
  setTimeout(function () {
    throw e;
  });
}
function Rl(e, t) {
  var n = t,
    r = 0;
  do {
    var i = n.nextSibling;
    if ((e.removeChild(n), i && i.nodeType === 8))
      if (((n = i.data), n === "/$")) {
        if (r === 0) {
          e.removeChild(i), oi(t);
          return;
        }
        r--;
      } else (n !== "$" && n !== "$?" && n !== "$!") || r++;
    n = i;
  } while (n);
  oi(t);
}
function Yt(e) {
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
function hc(e) {
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
var xr = Math.random().toString(36).slice(2),
  pt = "__reactFiber$" + xr,
  ci = "__reactProps$" + xr,
  Tt = "__reactContainer$" + xr,
  zs = "__reactEvents$" + xr,
  Qm = "__reactListeners$" + xr,
  Xm = "__reactHandles$" + xr;
function an(e) {
  var t = e[pt];
  if (t) return t;
  for (var n = e.parentNode; n; ) {
    if ((t = n[Tt] || n[pt])) {
      if (
        ((n = t.alternate),
        t.child !== null || (n !== null && n.child !== null))
      )
        for (e = hc(e); e !== null; ) {
          if ((n = e[pt])) return n;
          e = hc(e);
        }
      return t;
    }
    (e = n), (n = e.parentNode);
  }
  return null;
}
function ji(e) {
  return (
    (e = e[pt] || e[Tt]),
    !e || (e.tag !== 5 && e.tag !== 6 && e.tag !== 13 && e.tag !== 3) ? null : e
  );
}
function Wn(e) {
  if (e.tag === 5 || e.tag === 6) return e.stateNode;
  throw Error(M(33));
}
function rl(e) {
  return e[ci] || null;
}
var Os = [],
  Vn = -1;
function bt(e) {
  return { current: e };
}
function b(e) {
  0 > Vn || ((e.current = Os[Vn]), (Os[Vn] = null), Vn--);
}
function q(e, t) {
  Vn++, (Os[Vn] = e.current), (e.current = t);
}
var qt = {},
  Ee = bt(qt),
  Ae = bt(!1),
  xn = qt;
function lr(e, t) {
  var n = e.type.contextTypes;
  if (!n) return qt;
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
function De(e) {
  return (e = e.childContextTypes), e != null;
}
function Po() {
  b(Ae), b(Ee);
}
function pc(e, t, n) {
  if (Ee.current !== qt) throw Error(M(168));
  q(Ee, t), q(Ae, n);
}
function nh(e, t, n) {
  var r = e.stateNode;
  if (((t = t.childContextTypes), typeof r.getChildContext != "function"))
    return n;
  r = r.getChildContext();
  for (var i in r) if (!(i in t)) throw Error(M(108, $0(e) || "Unknown", i));
  return re({}, n, r);
}
function Co(e) {
  return (
    (e =
      ((e = e.stateNode) && e.__reactInternalMemoizedMergedChildContext) || qt),
    (xn = Ee.current),
    q(Ee, e),
    q(Ae, Ae.current),
    !0
  );
}
function mc(e, t, n) {
  var r = e.stateNode;
  if (!r) throw Error(M(169));
  n
    ? ((e = nh(e, t, xn)),
      (r.__reactInternalMemoizedMergedChildContext = e),
      b(Ae),
      b(Ee),
      q(Ee, e))
    : b(Ae),
    q(Ae, n);
}
var kt = null,
  il = !1,
  Fl = !1;
function rh(e) {
  kt === null ? (kt = [e]) : kt.push(e);
}
function Km(e) {
  (il = !0), rh(e);
}
function en() {
  if (!Fl && kt !== null) {
    Fl = !0;
    var e = 0,
      t = G;
    try {
      var n = kt;
      for (G = 1; e < n.length; e++) {
        var r = n[e];
        do r = r(!0);
        while (r !== null);
      }
      (kt = null), (il = !1);
    } catch (i) {
      throw (kt !== null && (kt = kt.slice(e + 1)), Td(Tu, en), i);
    } finally {
      (G = t), (Fl = !1);
    }
  }
  return null;
}
var Bn = [],
  Yn = 0,
  Mo = null,
  jo = 0,
  Qe = [],
  Xe = 0,
  vn = null,
  St = 1,
  Et = "";
function rn(e, t) {
  (Bn[Yn++] = jo), (Bn[Yn++] = Mo), (Mo = e), (jo = t);
}
function ih(e, t, n) {
  (Qe[Xe++] = St), (Qe[Xe++] = Et), (Qe[Xe++] = vn), (vn = e);
  var r = St;
  e = Et;
  var i = 32 - ut(r) - 1;
  (r &= ~(1 << i)), (n += 1);
  var o = 32 - ut(t) + i;
  if (30 < o) {
    var l = i - (i % 5);
    (o = (r & ((1 << l) - 1)).toString(32)),
      (r >>= l),
      (i -= l),
      (St = (1 << (32 - ut(t) + i)) | (n << i) | r),
      (Et = o + e);
  } else (St = (1 << o) | (n << i) | r), (Et = e);
}
function Ru(e) {
  e.return !== null && (rn(e, 1), ih(e, 1, 0));
}
function Fu(e) {
  for (; e === Mo; )
    (Mo = Bn[--Yn]), (Bn[Yn] = null), (jo = Bn[--Yn]), (Bn[Yn] = null);
  for (; e === vn; )
    (vn = Qe[--Xe]),
      (Qe[Xe] = null),
      (Et = Qe[--Xe]),
      (Qe[Xe] = null),
      (St = Qe[--Xe]),
      (Qe[Xe] = null);
}
var Ue = null,
  Ie = null,
  ee = !1,
  ot = null;
function oh(e, t) {
  var n = Ze(5, null, null, 0);
  (n.elementType = "DELETED"),
    (n.stateNode = t),
    (n.return = e),
    (t = e.deletions),
    t === null ? ((e.deletions = [n]), (e.flags |= 16)) : t.push(n);
}
function yc(e, t) {
  switch (e.tag) {
    case 5:
      var n = e.type;
      return (
        (t =
          t.nodeType !== 1 || n.toLowerCase() !== t.nodeName.toLowerCase()
            ? null
            : t),
        t !== null
          ? ((e.stateNode = t), (Ue = e), (Ie = Yt(t.firstChild)), !0)
          : !1
      );
    case 6:
      return (
        (t = e.pendingProps === "" || t.nodeType !== 3 ? null : t),
        t !== null ? ((e.stateNode = t), (Ue = e), (Ie = null), !0) : !1
      );
    case 13:
      return (
        (t = t.nodeType !== 8 ? null : t),
        t !== null
          ? ((n = vn !== null ? { id: St, overflow: Et } : null),
            (e.memoizedState = {
              dehydrated: t,
              treeContext: n,
              retryLane: 1073741824,
            }),
            (n = Ze(18, null, null, 0)),
            (n.stateNode = t),
            (n.return = e),
            (e.child = n),
            (Ue = e),
            (Ie = null),
            !0)
          : !1
      );
    default:
      return !1;
  }
}
function Rs(e) {
  return (e.mode & 1) !== 0 && (e.flags & 128) === 0;
}
function Fs(e) {
  if (ee) {
    var t = Ie;
    if (t) {
      var n = t;
      if (!yc(e, t)) {
        if (Rs(e)) throw Error(M(418));
        t = Yt(n.nextSibling);
        var r = Ue;
        t && yc(e, t)
          ? oh(r, n)
          : ((e.flags = (e.flags & -4097) | 2), (ee = !1), (Ue = e));
      }
    } else {
      if (Rs(e)) throw Error(M(418));
      (e.flags = (e.flags & -4097) | 2), (ee = !1), (Ue = e);
    }
  }
}
function gc(e) {
  for (e = e.return; e !== null && e.tag !== 5 && e.tag !== 3 && e.tag !== 13; )
    e = e.return;
  Ue = e;
}
function Yi(e) {
  if (e !== Ue) return !1;
  if (!ee) return gc(e), (ee = !0), !1;
  var t;
  if (
    ((t = e.tag !== 3) &&
      !(t = e.tag !== 5) &&
      ((t = e.type),
      (t = t !== "head" && t !== "body" && !As(e.type, e.memoizedProps))),
    t && (t = Ie))
  ) {
    if (Rs(e)) throw (lh(), Error(M(418)));
    for (; t; ) oh(e, t), (t = Yt(t.nextSibling));
  }
  if ((gc(e), e.tag === 13)) {
    if (((e = e.memoizedState), (e = e !== null ? e.dehydrated : null), !e))
      throw Error(M(317));
    e: {
      for (e = e.nextSibling, t = 0; e; ) {
        if (e.nodeType === 8) {
          var n = e.data;
          if (n === "/$") {
            if (t === 0) {
              Ie = Yt(e.nextSibling);
              break e;
            }
            t--;
          } else (n !== "$" && n !== "$!" && n !== "$?") || t++;
        }
        e = e.nextSibling;
      }
      Ie = null;
    }
  } else Ie = Ue ? Yt(e.stateNode.nextSibling) : null;
  return !0;
}
function lh() {
  for (var e = Ie; e; ) e = Yt(e.nextSibling);
}
function sr() {
  (Ie = Ue = null), (ee = !1);
}
function Iu(e) {
  ot === null ? (ot = [e]) : ot.push(e);
}
var Zm = At.ReactCurrentBatchConfig;
function Tr(e, t, n) {
  if (
    ((e = n.ref), e !== null && typeof e != "function" && typeof e != "object")
  ) {
    if (n._owner) {
      if (((n = n._owner), n)) {
        if (n.tag !== 1) throw Error(M(309));
        var r = n.stateNode;
      }
      if (!r) throw Error(M(147, e));
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
    if (typeof e != "string") throw Error(M(284));
    if (!n._owner) throw Error(M(290, e));
  }
  return e;
}
function Gi(e, t) {
  throw (
    ((e = Object.prototype.toString.call(t)),
    Error(
      M(
        31,
        e === "[object Object]"
          ? "object with keys {" + Object.keys(t).join(", ") + "}"
          : e
      )
    ))
  );
}
function xc(e) {
  var t = e._init;
  return t(e._payload);
}
function sh(e) {
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
      ? ((p = Yl(y, m.mode, w)), (p.return = m), p)
      : ((p = i(p, y)), (p.return = m), p);
  }
  function u(m, p, y, w) {
    var k = y.type;
    return k === Fn
      ? c(m, p, y.props.children, w, y.key)
      : p !== null &&
        (p.elementType === k ||
          (typeof k == "object" &&
            k !== null &&
            k.$$typeof === zt &&
            xc(k) === p.type))
      ? ((w = i(p, y.props)), (w.ref = Tr(m, p, y)), (w.return = m), w)
      : ((w = ho(y.type, y.key, y.props, null, m.mode, w)),
        (w.ref = Tr(m, p, y)),
        (w.return = m),
        w);
  }
  function a(m, p, y, w) {
    return p === null ||
      p.tag !== 4 ||
      p.stateNode.containerInfo !== y.containerInfo ||
      p.stateNode.implementation !== y.implementation
      ? ((p = Gl(y, m.mode, w)), (p.return = m), p)
      : ((p = i(p, y.children || [])), (p.return = m), p);
  }
  function c(m, p, y, w, k) {
    return p === null || p.tag !== 7
      ? ((p = pn(y, m.mode, w, k)), (p.return = m), p)
      : ((p = i(p, y)), (p.return = m), p);
  }
  function f(m, p, y) {
    if ((typeof p == "string" && p !== "") || typeof p == "number")
      return (p = Yl("" + p, m.mode, y)), (p.return = m), p;
    if (typeof p == "object" && p !== null) {
      switch (p.$$typeof) {
        case zi:
          return (
            (y = ho(p.type, p.key, p.props, null, m.mode, y)),
            (y.ref = Tr(m, null, p)),
            (y.return = m),
            y
          );
        case Rn:
          return (p = Gl(p, m.mode, y)), (p.return = m), p;
        case zt:
          var w = p._init;
          return f(m, w(p._payload), y);
      }
      if (Rr(p) || Er(p))
        return (p = pn(p, m.mode, y, null)), (p.return = m), p;
      Gi(m, p);
    }
    return null;
  }
  function d(m, p, y, w) {
    var k = p !== null ? p.key : null;
    if ((typeof y == "string" && y !== "") || typeof y == "number")
      return k !== null ? null : s(m, p, "" + y, w);
    if (typeof y == "object" && y !== null) {
      switch (y.$$typeof) {
        case zi:
          return y.key === k ? u(m, p, y, w) : null;
        case Rn:
          return y.key === k ? a(m, p, y, w) : null;
        case zt:
          return (k = y._init), d(m, p, k(y._payload), w);
      }
      if (Rr(y) || Er(y)) return k !== null ? null : c(m, p, y, w, null);
      Gi(m, y);
    }
    return null;
  }
  function x(m, p, y, w, k) {
    if ((typeof w == "string" && w !== "") || typeof w == "number")
      return (m = m.get(y) || null), s(p, m, "" + w, k);
    if (typeof w == "object" && w !== null) {
      switch (w.$$typeof) {
        case zi:
          return (m = m.get(w.key === null ? y : w.key) || null), u(p, m, w, k);
        case Rn:
          return (m = m.get(w.key === null ? y : w.key) || null), a(p, m, w, k);
        case zt:
          var S = w._init;
          return x(m, p, y, S(w._payload), k);
      }
      if (Rr(w) || Er(w)) return (m = m.get(y) || null), c(p, m, w, k, null);
      Gi(p, w);
    }
    return null;
  }
  function g(m, p, y, w) {
    for (
      var k = null, S = null, P = p, C = (p = 0), D = null;
      P !== null && C < y.length;
      C++
    ) {
      P.index > C ? ((D = P), (P = null)) : (D = P.sibling);
      var O = d(m, P, y[C], w);
      if (O === null) {
        P === null && (P = D);
        break;
      }
      e && P && O.alternate === null && t(m, P),
        (p = o(O, p, C)),
        S === null ? (k = O) : (S.sibling = O),
        (S = O),
        (P = D);
    }
    if (C === y.length) return n(m, P), ee && rn(m, C), k;
    if (P === null) {
      for (; C < y.length; C++)
        (P = f(m, y[C], w)),
          P !== null &&
            ((p = o(P, p, C)), S === null ? (k = P) : (S.sibling = P), (S = P));
      return ee && rn(m, C), k;
    }
    for (P = r(m, P); C < y.length; C++)
      (D = x(P, m, C, y[C], w)),
        D !== null &&
          (e && D.alternate !== null && P.delete(D.key === null ? C : D.key),
          (p = o(D, p, C)),
          S === null ? (k = D) : (S.sibling = D),
          (S = D));
    return (
      e &&
        P.forEach(function (N) {
          return t(m, N);
        }),
      ee && rn(m, C),
      k
    );
  }
  function v(m, p, y, w) {
    var k = Er(y);
    if (typeof k != "function") throw Error(M(150));
    if (((y = k.call(y)), y == null)) throw Error(M(151));
    for (
      var S = (k = null), P = p, C = (p = 0), D = null, O = y.next();
      P !== null && !O.done;
      C++, O = y.next()
    ) {
      P.index > C ? ((D = P), (P = null)) : (D = P.sibling);
      var N = d(m, P, O.value, w);
      if (N === null) {
        P === null && (P = D);
        break;
      }
      e && P && N.alternate === null && t(m, P),
        (p = o(N, p, C)),
        S === null ? (k = N) : (S.sibling = N),
        (S = N),
        (P = D);
    }
    if (O.done) return n(m, P), ee && rn(m, C), k;
    if (P === null) {
      for (; !O.done; C++, O = y.next())
        (O = f(m, O.value, w)),
          O !== null &&
            ((p = o(O, p, C)), S === null ? (k = O) : (S.sibling = O), (S = O));
      return ee && rn(m, C), k;
    }
    for (P = r(m, P); !O.done; C++, O = y.next())
      (O = x(P, m, C, O.value, w)),
        O !== null &&
          (e && O.alternate !== null && P.delete(O.key === null ? C : O.key),
          (p = o(O, p, C)),
          S === null ? (k = O) : (S.sibling = O),
          (S = O));
    return (
      e &&
        P.forEach(function (H) {
          return t(m, H);
        }),
      ee && rn(m, C),
      k
    );
  }
  function E(m, p, y, w) {
    if (
      (typeof y == "object" &&
        y !== null &&
        y.type === Fn &&
        y.key === null &&
        (y = y.props.children),
      typeof y == "object" && y !== null)
    ) {
      switch (y.$$typeof) {
        case zi:
          e: {
            for (var k = y.key, S = p; S !== null; ) {
              if (S.key === k) {
                if (((k = y.type), k === Fn)) {
                  if (S.tag === 7) {
                    n(m, S.sibling),
                      (p = i(S, y.props.children)),
                      (p.return = m),
                      (m = p);
                    break e;
                  }
                } else if (
                  S.elementType === k ||
                  (typeof k == "object" &&
                    k !== null &&
                    k.$$typeof === zt &&
                    xc(k) === S.type)
                ) {
                  n(m, S.sibling),
                    (p = i(S, y.props)),
                    (p.ref = Tr(m, S, y)),
                    (p.return = m),
                    (m = p);
                  break e;
                }
                n(m, S);
                break;
              } else t(m, S);
              S = S.sibling;
            }
            y.type === Fn
              ? ((p = pn(y.props.children, m.mode, w, y.key)),
                (p.return = m),
                (m = p))
              : ((w = ho(y.type, y.key, y.props, null, m.mode, w)),
                (w.ref = Tr(m, p, y)),
                (w.return = m),
                (m = w));
          }
          return l(m);
        case Rn:
          e: {
            for (S = y.key; p !== null; ) {
              if (p.key === S)
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
            (p = Gl(y, m.mode, w)), (p.return = m), (m = p);
          }
          return l(m);
        case zt:
          return (S = y._init), E(m, p, S(y._payload), w);
      }
      if (Rr(y)) return g(m, p, y, w);
      if (Er(y)) return v(m, p, y, w);
      Gi(m, y);
    }
    return (typeof y == "string" && y !== "") || typeof y == "number"
      ? ((y = "" + y),
        p !== null && p.tag === 6
          ? (n(m, p.sibling), (p = i(p, y)), (p.return = m), (m = p))
          : (n(m, p), (p = Yl(y, m.mode, w)), (p.return = m), (m = p)),
        l(m))
      : n(m, p);
  }
  return E;
}
var ur = sh(!0),
  uh = sh(!1),
  To = bt(null),
  _o = null,
  Gn = null,
  Uu = null;
function Hu() {
  Uu = Gn = _o = null;
}
function Wu(e) {
  var t = To.current;
  b(To), (e._currentValue = t);
}
function Is(e, t, n) {
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
function nr(e, t) {
  (_o = e),
    (Uu = Gn = null),
    (e = e.dependencies),
    e !== null &&
      e.firstContext !== null &&
      (e.lanes & t && (Ne = !0), (e.firstContext = null));
}
function be(e) {
  var t = e._currentValue;
  if (Uu !== e)
    if (((e = { context: e, memoizedValue: t, next: null }), Gn === null)) {
      if (_o === null) throw Error(M(308));
      (Gn = e), (_o.dependencies = { lanes: 0, firstContext: e });
    } else Gn = Gn.next = e;
  return t;
}
var cn = null;
function Vu(e) {
  cn === null ? (cn = [e]) : cn.push(e);
}
function ah(e, t, n, r) {
  var i = t.interleaved;
  return (
    i === null ? ((n.next = n), Vu(t)) : ((n.next = i.next), (i.next = n)),
    (t.interleaved = n),
    _t(e, r)
  );
}
function _t(e, t) {
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
var Ot = !1;
function Bu(e) {
  e.updateQueue = {
    baseState: e.memoizedState,
    firstBaseUpdate: null,
    lastBaseUpdate: null,
    shared: { pending: null, interleaved: null, lanes: 0 },
    effects: null,
  };
}
function ch(e, t) {
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
  if (((r = r.shared), W & 2)) {
    var i = r.pending;
    return (
      i === null ? (t.next = t) : ((t.next = i.next), (i.next = t)),
      (r.pending = t),
      _t(e, n)
    );
  }
  return (
    (i = r.interleaved),
    i === null ? ((t.next = t), Vu(r)) : ((t.next = i.next), (i.next = t)),
    (r.interleaved = t),
    _t(e, n)
  );
}
function lo(e, t, n) {
  if (
    ((t = t.updateQueue), t !== null && ((t = t.shared), (n & 4194240) !== 0))
  ) {
    var r = t.lanes;
    (r &= e.pendingLanes), (n |= r), (t.lanes = n), _u(e, n);
  }
}
function vc(e, t) {
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
function Lo(e, t, n, r) {
  var i = e.updateQueue;
  Ot = !1;
  var o = i.firstBaseUpdate,
    l = i.lastBaseUpdate,
    s = i.shared.pending;
  if (s !== null) {
    i.shared.pending = null;
    var u = s,
      a = u.next;
    (u.next = null), l === null ? (o = a) : (l.next = a), (l = u);
    var c = e.alternate;
    c !== null &&
      ((c = c.updateQueue),
      (s = c.lastBaseUpdate),
      s !== l &&
        (s === null ? (c.firstBaseUpdate = a) : (s.next = a),
        (c.lastBaseUpdate = u)));
  }
  if (o !== null) {
    var f = i.baseState;
    (l = 0), (c = a = u = null), (s = o);
    do {
      var d = s.lane,
        x = s.eventTime;
      if ((r & d) === d) {
        c !== null &&
          (c = c.next =
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
          switch (((d = t), (x = n), v.tag)) {
            case 1:
              if (((g = v.payload), typeof g == "function")) {
                f = g.call(x, f, d);
                break e;
              }
              f = g;
              break e;
            case 3:
              g.flags = (g.flags & -65537) | 128;
            case 0:
              if (
                ((g = v.payload),
                (d = typeof g == "function" ? g.call(x, f, d) : g),
                d == null)
              )
                break e;
              f = re({}, f, d);
              break e;
            case 2:
              Ot = !0;
          }
        }
        s.callback !== null &&
          s.lane !== 0 &&
          ((e.flags |= 64),
          (d = i.effects),
          d === null ? (i.effects = [s]) : d.push(s));
      } else
        (x = {
          eventTime: x,
          lane: d,
          tag: s.tag,
          payload: s.payload,
          callback: s.callback,
          next: null,
        }),
          c === null ? ((a = c = x), (u = f)) : (c = c.next = x),
          (l |= d);
      if (((s = s.next), s === null)) {
        if (((s = i.shared.pending), s === null)) break;
        (d = s),
          (s = d.next),
          (d.next = null),
          (i.lastBaseUpdate = d),
          (i.shared.pending = null);
      }
    } while (!0);
    if (
      (c === null && (u = f),
      (i.baseState = u),
      (i.firstBaseUpdate = a),
      (i.lastBaseUpdate = c),
      (t = i.shared.interleaved),
      t !== null)
    ) {
      i = t;
      do (l |= i.lane), (i = i.next);
      while (i !== t);
    } else o === null && (i.shared.lanes = 0);
    (kn |= l), (e.lanes = l), (e.memoizedState = f);
  }
}
function wc(e, t, n) {
  if (((e = t.effects), (t.effects = null), e !== null))
    for (t = 0; t < e.length; t++) {
      var r = e[t],
        i = r.callback;
      if (i !== null) {
        if (((r.callback = null), (r = n), typeof i != "function"))
          throw Error(M(191, i));
        i.call(r);
      }
    }
}
var Ti = {},
  yt = bt(Ti),
  fi = bt(Ti),
  di = bt(Ti);
function fn(e) {
  if (e === Ti) throw Error(M(174));
  return e;
}
function Yu(e, t) {
  switch ((q(di, t), q(fi, e), q(yt, Ti), (e = t.nodeType), e)) {
    case 9:
    case 11:
      t = (t = t.documentElement) ? t.namespaceURI : vs(null, "");
      break;
    default:
      (e = e === 8 ? t.parentNode : t),
        (t = e.namespaceURI || null),
        (e = e.tagName),
        (t = vs(t, e));
  }
  b(yt), q(yt, t);
}
function ar() {
  b(yt), b(fi), b(di);
}
function fh(e) {
  fn(di.current);
  var t = fn(yt.current),
    n = vs(t, e.type);
  t !== n && (q(fi, e), q(yt, n));
}
function Gu(e) {
  fi.current === e && (b(yt), b(fi));
}
var te = bt(0);
function No(e) {
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
var Il = [];
function Qu() {
  for (var e = 0; e < Il.length; e++)
    Il[e]._workInProgressVersionPrimary = null;
  Il.length = 0;
}
var so = At.ReactCurrentDispatcher,
  Ul = At.ReactCurrentBatchConfig,
  wn = 0,
  ne = null,
  ae = null,
  he = null,
  $o = !1,
  Kr = !1,
  hi = 0,
  qm = 0;
function ve() {
  throw Error(M(321));
}
function Xu(e, t) {
  if (t === null) return !1;
  for (var n = 0; n < t.length && n < e.length; n++)
    if (!ct(e[n], t[n])) return !1;
  return !0;
}
function Ku(e, t, n, r, i, o) {
  if (
    ((wn = o),
    (ne = t),
    (t.memoizedState = null),
    (t.updateQueue = null),
    (t.lanes = 0),
    (so.current = e === null || e.memoizedState === null ? ty : ny),
    (e = n(r, i)),
    Kr)
  ) {
    o = 0;
    do {
      if (((Kr = !1), (hi = 0), 25 <= o)) throw Error(M(301));
      (o += 1),
        (he = ae = null),
        (t.updateQueue = null),
        (so.current = ry),
        (e = n(r, i));
    } while (Kr);
  }
  if (
    ((so.current = Ao),
    (t = ae !== null && ae.next !== null),
    (wn = 0),
    (he = ae = ne = null),
    ($o = !1),
    t)
  )
    throw Error(M(300));
  return e;
}
function Zu() {
  var e = hi !== 0;
  return (hi = 0), e;
}
function dt() {
  var e = {
    memoizedState: null,
    baseState: null,
    baseQueue: null,
    queue: null,
    next: null,
  };
  return he === null ? (ne.memoizedState = he = e) : (he = he.next = e), he;
}
function et() {
  if (ae === null) {
    var e = ne.alternate;
    e = e !== null ? e.memoizedState : null;
  } else e = ae.next;
  var t = he === null ? ne.memoizedState : he.next;
  if (t !== null) (he = t), (ae = e);
  else {
    if (e === null) throw Error(M(310));
    (ae = e),
      (e = {
        memoizedState: ae.memoizedState,
        baseState: ae.baseState,
        baseQueue: ae.baseQueue,
        queue: ae.queue,
        next: null,
      }),
      he === null ? (ne.memoizedState = he = e) : (he = he.next = e);
  }
  return he;
}
function pi(e, t) {
  return typeof t == "function" ? t(e) : t;
}
function Hl(e) {
  var t = et(),
    n = t.queue;
  if (n === null) throw Error(M(311));
  n.lastRenderedReducer = e;
  var r = ae,
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
      var c = a.lane;
      if ((wn & c) === c)
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
        var f = {
          lane: c,
          action: a.action,
          hasEagerState: a.hasEagerState,
          eagerState: a.eagerState,
          next: null,
        };
        u === null ? ((s = u = f), (l = r)) : (u = u.next = f),
          (ne.lanes |= c),
          (kn |= c);
      }
      a = a.next;
    } while (a !== null && a !== o);
    u === null ? (l = r) : (u.next = s),
      ct(r, t.memoizedState) || (Ne = !0),
      (t.memoizedState = r),
      (t.baseState = l),
      (t.baseQueue = u),
      (n.lastRenderedState = r);
  }
  if (((e = n.interleaved), e !== null)) {
    i = e;
    do (o = i.lane), (ne.lanes |= o), (kn |= o), (i = i.next);
    while (i !== e);
  } else i === null && (n.lanes = 0);
  return [t.memoizedState, n.dispatch];
}
function Wl(e) {
  var t = et(),
    n = t.queue;
  if (n === null) throw Error(M(311));
  n.lastRenderedReducer = e;
  var r = n.dispatch,
    i = n.pending,
    o = t.memoizedState;
  if (i !== null) {
    n.pending = null;
    var l = (i = i.next);
    do (o = e(o, l.action)), (l = l.next);
    while (l !== i);
    ct(o, t.memoizedState) || (Ne = !0),
      (t.memoizedState = o),
      t.baseQueue === null && (t.baseState = o),
      (n.lastRenderedState = o);
  }
  return [o, r];
}
function dh() {}
function hh(e, t) {
  var n = ne,
    r = et(),
    i = t(),
    o = !ct(r.memoizedState, i);
  if (
    (o && ((r.memoizedState = i), (Ne = !0)),
    (r = r.queue),
    qu(yh.bind(null, n, r, e), [e]),
    r.getSnapshot !== t || o || (he !== null && he.memoizedState.tag & 1))
  ) {
    if (
      ((n.flags |= 2048),
      mi(9, mh.bind(null, n, r, i, t), void 0, null),
      pe === null)
    )
      throw Error(M(349));
    wn & 30 || ph(n, t, i);
  }
  return i;
}
function ph(e, t, n) {
  (e.flags |= 16384),
    (e = { getSnapshot: t, value: n }),
    (t = ne.updateQueue),
    t === null
      ? ((t = { lastEffect: null, stores: null }),
        (ne.updateQueue = t),
        (t.stores = [e]))
      : ((n = t.stores), n === null ? (t.stores = [e]) : n.push(e));
}
function mh(e, t, n, r) {
  (t.value = n), (t.getSnapshot = r), gh(t) && xh(e);
}
function yh(e, t, n) {
  return n(function () {
    gh(t) && xh(e);
  });
}
function gh(e) {
  var t = e.getSnapshot;
  e = e.value;
  try {
    var n = t();
    return !ct(e, n);
  } catch {
    return !0;
  }
}
function xh(e) {
  var t = _t(e, 1);
  t !== null && at(t, e, 1, -1);
}
function kc(e) {
  var t = dt();
  return (
    typeof e == "function" && (e = e()),
    (t.memoizedState = t.baseState = e),
    (e = {
      pending: null,
      interleaved: null,
      lanes: 0,
      dispatch: null,
      lastRenderedReducer: pi,
      lastRenderedState: e,
    }),
    (t.queue = e),
    (e = e.dispatch = ey.bind(null, ne, e)),
    [t.memoizedState, e]
  );
}
function mi(e, t, n, r) {
  return (
    (e = { tag: e, create: t, destroy: n, deps: r, next: null }),
    (t = ne.updateQueue),
    t === null
      ? ((t = { lastEffect: null, stores: null }),
        (ne.updateQueue = t),
        (t.lastEffect = e.next = e))
      : ((n = t.lastEffect),
        n === null
          ? (t.lastEffect = e.next = e)
          : ((r = n.next), (n.next = e), (e.next = r), (t.lastEffect = e))),
    e
  );
}
function vh() {
  return et().memoizedState;
}
function uo(e, t, n, r) {
  var i = dt();
  (ne.flags |= e),
    (i.memoizedState = mi(1 | t, n, void 0, r === void 0 ? null : r));
}
function ol(e, t, n, r) {
  var i = et();
  r = r === void 0 ? null : r;
  var o = void 0;
  if (ae !== null) {
    var l = ae.memoizedState;
    if (((o = l.destroy), r !== null && Xu(r, l.deps))) {
      i.memoizedState = mi(t, n, o, r);
      return;
    }
  }
  (ne.flags |= e), (i.memoizedState = mi(1 | t, n, o, r));
}
function Sc(e, t) {
  return uo(8390656, 8, e, t);
}
function qu(e, t) {
  return ol(2048, 8, e, t);
}
function wh(e, t) {
  return ol(4, 2, e, t);
}
function kh(e, t) {
  return ol(4, 4, e, t);
}
function Sh(e, t) {
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
    (n = n != null ? n.concat([e]) : null), ol(4, 4, Sh.bind(null, t, e), n)
  );
}
function Ju() {}
function Ph(e, t) {
  var n = et();
  t = t === void 0 ? null : t;
  var r = n.memoizedState;
  return r !== null && t !== null && Xu(t, r[1])
    ? r[0]
    : ((n.memoizedState = [e, t]), e);
}
function Ch(e, t) {
  var n = et();
  t = t === void 0 ? null : t;
  var r = n.memoizedState;
  return r !== null && t !== null && Xu(t, r[1])
    ? r[0]
    : ((e = e()), (n.memoizedState = [e, t]), e);
}
function Mh(e, t, n) {
  return wn & 21
    ? (ct(n, t) || ((n = Nd()), (ne.lanes |= n), (kn |= n), (e.baseState = !0)),
      t)
    : (e.baseState && ((e.baseState = !1), (Ne = !0)), (e.memoizedState = n));
}
function Jm(e, t) {
  var n = G;
  (G = n !== 0 && 4 > n ? n : 4), e(!0);
  var r = Ul.transition;
  Ul.transition = {};
  try {
    e(!1), t();
  } finally {
    (G = n), (Ul.transition = r);
  }
}
function jh() {
  return et().memoizedState;
}
function bm(e, t, n) {
  var r = Xt(e);
  if (
    ((n = {
      lane: r,
      action: n,
      hasEagerState: !1,
      eagerState: null,
      next: null,
    }),
    Th(e))
  )
    _h(t, n);
  else if (((n = ah(e, t, n, r)), n !== null)) {
    var i = Ce();
    at(n, e, r, i), Lh(n, t, r);
  }
}
function ey(e, t, n) {
  var r = Xt(e),
    i = { lane: r, action: n, hasEagerState: !1, eagerState: null, next: null };
  if (Th(e)) _h(t, i);
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
        if (((i.hasEagerState = !0), (i.eagerState = s), ct(s, l))) {
          var u = t.interleaved;
          u === null
            ? ((i.next = i), Vu(t))
            : ((i.next = u.next), (u.next = i)),
            (t.interleaved = i);
          return;
        }
      } catch {
      } finally {
      }
    (n = ah(e, t, i, r)),
      n !== null && ((i = Ce()), at(n, e, r, i), Lh(n, t, r));
  }
}
function Th(e) {
  var t = e.alternate;
  return e === ne || (t !== null && t === ne);
}
function _h(e, t) {
  Kr = $o = !0;
  var n = e.pending;
  n === null ? (t.next = t) : ((t.next = n.next), (n.next = t)),
    (e.pending = t);
}
function Lh(e, t, n) {
  if (n & 4194240) {
    var r = t.lanes;
    (r &= e.pendingLanes), (n |= r), (t.lanes = n), _u(e, n);
  }
}
var Ao = {
    readContext: be,
    useCallback: ve,
    useContext: ve,
    useEffect: ve,
    useImperativeHandle: ve,
    useInsertionEffect: ve,
    useLayoutEffect: ve,
    useMemo: ve,
    useReducer: ve,
    useRef: ve,
    useState: ve,
    useDebugValue: ve,
    useDeferredValue: ve,
    useTransition: ve,
    useMutableSource: ve,
    useSyncExternalStore: ve,
    useId: ve,
    unstable_isNewReconciler: !1,
  },
  ty = {
    readContext: be,
    useCallback: function (e, t) {
      return (dt().memoizedState = [e, t === void 0 ? null : t]), e;
    },
    useContext: be,
    useEffect: Sc,
    useImperativeHandle: function (e, t, n) {
      return (
        (n = n != null ? n.concat([e]) : null),
        uo(4194308, 4, Sh.bind(null, t, e), n)
      );
    },
    useLayoutEffect: function (e, t) {
      return uo(4194308, 4, e, t);
    },
    useInsertionEffect: function (e, t) {
      return uo(4, 2, e, t);
    },
    useMemo: function (e, t) {
      var n = dt();
      return (
        (t = t === void 0 ? null : t), (e = e()), (n.memoizedState = [e, t]), e
      );
    },
    useReducer: function (e, t, n) {
      var r = dt();
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
        (e = e.dispatch = bm.bind(null, ne, e)),
        [r.memoizedState, e]
      );
    },
    useRef: function (e) {
      var t = dt();
      return (e = { current: e }), (t.memoizedState = e);
    },
    useState: kc,
    useDebugValue: Ju,
    useDeferredValue: function (e) {
      return (dt().memoizedState = e);
    },
    useTransition: function () {
      var e = kc(!1),
        t = e[0];
      return (e = Jm.bind(null, e[1])), (dt().memoizedState = e), [t, e];
    },
    useMutableSource: function () {},
    useSyncExternalStore: function (e, t, n) {
      var r = ne,
        i = dt();
      if (ee) {
        if (n === void 0) throw Error(M(407));
        n = n();
      } else {
        if (((n = t()), pe === null)) throw Error(M(349));
        wn & 30 || ph(r, t, n);
      }
      i.memoizedState = n;
      var o = { value: n, getSnapshot: t };
      return (
        (i.queue = o),
        Sc(yh.bind(null, r, o, e), [e]),
        (r.flags |= 2048),
        mi(9, mh.bind(null, r, o, n, t), void 0, null),
        n
      );
    },
    useId: function () {
      var e = dt(),
        t = pe.identifierPrefix;
      if (ee) {
        var n = Et,
          r = St;
        (n = (r & ~(1 << (32 - ut(r) - 1))).toString(32) + n),
          (t = ":" + t + "R" + n),
          (n = hi++),
          0 < n && (t += "H" + n.toString(32)),
          (t += ":");
      } else (n = qm++), (t = ":" + t + "r" + n.toString(32) + ":");
      return (e.memoizedState = t);
    },
    unstable_isNewReconciler: !1,
  },
  ny = {
    readContext: be,
    useCallback: Ph,
    useContext: be,
    useEffect: qu,
    useImperativeHandle: Eh,
    useInsertionEffect: wh,
    useLayoutEffect: kh,
    useMemo: Ch,
    useReducer: Hl,
    useRef: vh,
    useState: function () {
      return Hl(pi);
    },
    useDebugValue: Ju,
    useDeferredValue: function (e) {
      var t = et();
      return Mh(t, ae.memoizedState, e);
    },
    useTransition: function () {
      var e = Hl(pi)[0],
        t = et().memoizedState;
      return [e, t];
    },
    useMutableSource: dh,
    useSyncExternalStore: hh,
    useId: jh,
    unstable_isNewReconciler: !1,
  },
  ry = {
    readContext: be,
    useCallback: Ph,
    useContext: be,
    useEffect: qu,
    useImperativeHandle: Eh,
    useInsertionEffect: wh,
    useLayoutEffect: kh,
    useMemo: Ch,
    useReducer: Wl,
    useRef: vh,
    useState: function () {
      return Wl(pi);
    },
    useDebugValue: Ju,
    useDeferredValue: function (e) {
      var t = et();
      return ae === null ? (t.memoizedState = e) : Mh(t, ae.memoizedState, e);
    },
    useTransition: function () {
      var e = Wl(pi)[0],
        t = et().memoizedState;
      return [e, t];
    },
    useMutableSource: dh,
    useSyncExternalStore: hh,
    useId: jh,
    unstable_isNewReconciler: !1,
  };
function rt(e, t) {
  if (e && e.defaultProps) {
    (t = re({}, t)), (e = e.defaultProps);
    for (var n in e) t[n] === void 0 && (t[n] = e[n]);
    return t;
  }
  return t;
}
function Us(e, t, n, r) {
  (t = e.memoizedState),
    (n = n(r, t)),
    (n = n == null ? t : re({}, t, n)),
    (e.memoizedState = n),
    e.lanes === 0 && (e.updateQueue.baseState = n);
}
var ll = {
  isMounted: function (e) {
    return (e = e._reactInternals) ? Tn(e) === e : !1;
  },
  enqueueSetState: function (e, t, n) {
    e = e._reactInternals;
    var r = Ce(),
      i = Xt(e),
      o = Mt(r, i);
    (o.payload = t),
      n != null && (o.callback = n),
      (t = Gt(e, o, i)),
      t !== null && (at(t, e, i, r), lo(t, e, i));
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
      t !== null && (at(t, e, i, r), lo(t, e, i));
  },
  enqueueForceUpdate: function (e, t) {
    e = e._reactInternals;
    var n = Ce(),
      r = Xt(e),
      i = Mt(n, r);
    (i.tag = 2),
      t != null && (i.callback = t),
      (t = Gt(e, i, r)),
      t !== null && (at(t, e, r, n), lo(t, e, r));
  },
};
function Ec(e, t, n, r, i, o, l) {
  return (
    (e = e.stateNode),
    typeof e.shouldComponentUpdate == "function"
      ? e.shouldComponentUpdate(r, o, l)
      : t.prototype && t.prototype.isPureReactComponent
      ? !si(n, r) || !si(i, o)
      : !0
  );
}
function Nh(e, t, n) {
  var r = !1,
    i = qt,
    o = t.contextType;
  return (
    typeof o == "object" && o !== null
      ? (o = be(o))
      : ((i = De(t) ? xn : Ee.current),
        (r = t.contextTypes),
        (o = (r = r != null) ? lr(e, i) : qt)),
    (t = new t(n, o)),
    (e.memoizedState = t.state !== null && t.state !== void 0 ? t.state : null),
    (t.updater = ll),
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
    t.state !== e && ll.enqueueReplaceState(t, t.state, null);
}
function Hs(e, t, n, r) {
  var i = e.stateNode;
  (i.props = n), (i.state = e.memoizedState), (i.refs = {}), Bu(e);
  var o = t.contextType;
  typeof o == "object" && o !== null
    ? (i.context = be(o))
    : ((o = De(t) ? xn : Ee.current), (i.context = lr(e, o))),
    (i.state = e.memoizedState),
    (o = t.getDerivedStateFromProps),
    typeof o == "function" && (Us(e, t, o, n), (i.state = e.memoizedState)),
    typeof t.getDerivedStateFromProps == "function" ||
      typeof i.getSnapshotBeforeUpdate == "function" ||
      (typeof i.UNSAFE_componentWillMount != "function" &&
        typeof i.componentWillMount != "function") ||
      ((t = i.state),
      typeof i.componentWillMount == "function" && i.componentWillMount(),
      typeof i.UNSAFE_componentWillMount == "function" &&
        i.UNSAFE_componentWillMount(),
      t !== i.state && ll.enqueueReplaceState(i, i.state, null),
      Lo(e, n, i, r),
      (i.state = e.memoizedState)),
    typeof i.componentDidMount == "function" && (e.flags |= 4194308);
}
function cr(e, t) {
  try {
    var n = "",
      r = t;
    do (n += N0(r)), (r = r.return);
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
function Vl(e, t, n) {
  return { value: e, source: null, stack: n ?? null, digest: t ?? null };
}
function Ws(e, t) {
  try {
    console.error(t.value);
  } catch (n) {
    setTimeout(function () {
      throw n;
    });
  }
}
var iy = typeof WeakMap == "function" ? WeakMap : Map;
function $h(e, t, n) {
  (n = Mt(-1, n)), (n.tag = 3), (n.payload = { element: null });
  var r = t.value;
  return (
    (n.callback = function () {
      zo || ((zo = !0), (Js = r)), Ws(e, t);
    }),
    n
  );
}
function Ah(e, t, n) {
  (n = Mt(-1, n)), (n.tag = 3);
  var r = e.type.getDerivedStateFromError;
  if (typeof r == "function") {
    var i = t.value;
    (n.payload = function () {
      return r(i);
    }),
      (n.callback = function () {
        Ws(e, t);
      });
  }
  var o = e.stateNode;
  return (
    o !== null &&
      typeof o.componentDidCatch == "function" &&
      (n.callback = function () {
        Ws(e, t),
          typeof r != "function" &&
            (Qt === null ? (Qt = new Set([this])) : Qt.add(this));
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
    r = e.pingCache = new iy();
    var i = new Set();
    r.set(t, i);
  } else (i = r.get(t)), i === void 0 && ((i = new Set()), r.set(t, i));
  i.has(n) || (i.add(n), (e = xy.bind(null, e, t, n)), t.then(e, e));
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
function jc(e, t, n, r, i) {
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
var oy = At.ReactCurrentOwner,
  Ne = !1;
function Pe(e, t, n, r) {
  t.child = e === null ? uh(t, null, n, r) : ur(t, e.child, n, r);
}
function Tc(e, t, n, r, i) {
  n = n.render;
  var o = t.ref;
  return (
    nr(t, i),
    (r = Ku(e, t, n, r, o, i)),
    (n = Zu()),
    e !== null && !Ne
      ? ((t.updateQueue = e.updateQueue),
        (t.flags &= -2053),
        (e.lanes &= ~i),
        Lt(e, t, i))
      : (ee && n && Ru(t), (t.flags |= 1), Pe(e, t, r, i), t.child)
  );
}
function _c(e, t, n, r, i) {
  if (e === null) {
    var o = n.type;
    return typeof o == "function" &&
      !la(o) &&
      o.defaultProps === void 0 &&
      n.compare === null &&
      n.defaultProps === void 0
      ? ((t.tag = 15), (t.type = o), Dh(e, t, o, r, i))
      : ((e = ho(n.type, null, r, t, t.mode, i)),
        (e.ref = t.ref),
        (e.return = t),
        (t.child = e));
  }
  if (((o = e.child), !(e.lanes & i))) {
    var l = o.memoizedProps;
    if (
      ((n = n.compare), (n = n !== null ? n : si), n(l, r) && e.ref === t.ref)
    )
      return Lt(e, t, i);
  }
  return (
    (t.flags |= 1),
    (e = Kt(o, r)),
    (e.ref = t.ref),
    (e.return = t),
    (t.child = e)
  );
}
function Dh(e, t, n, r, i) {
  if (e !== null) {
    var o = e.memoizedProps;
    if (si(o, r) && e.ref === t.ref)
      if (((Ne = !1), (t.pendingProps = r = o), (e.lanes & i) !== 0))
        e.flags & 131072 && (Ne = !0);
      else return (t.lanes = e.lanes), Lt(e, t, i);
  }
  return Vs(e, t, n, r, i);
}
function zh(e, t, n) {
  var r = t.pendingProps,
    i = r.children,
    o = e !== null ? e.memoizedState : null;
  if (r.mode === "hidden")
    if (!(t.mode & 1))
      (t.memoizedState = { baseLanes: 0, cachePool: null, transitions: null }),
        q(Xn, Fe),
        (Fe |= n);
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
          q(Xn, Fe),
          (Fe |= e),
          null
        );
      (t.memoizedState = { baseLanes: 0, cachePool: null, transitions: null }),
        (r = o !== null ? o.baseLanes : n),
        q(Xn, Fe),
        (Fe |= r);
    }
  else
    o !== null ? ((r = o.baseLanes | n), (t.memoizedState = null)) : (r = n),
      q(Xn, Fe),
      (Fe |= r);
  return Pe(e, t, i, n), t.child;
}
function Oh(e, t) {
  var n = t.ref;
  ((e === null && n !== null) || (e !== null && e.ref !== n)) &&
    ((t.flags |= 512), (t.flags |= 2097152));
}
function Vs(e, t, n, r, i) {
  var o = De(n) ? xn : Ee.current;
  return (
    (o = lr(t, o)),
    nr(t, i),
    (n = Ku(e, t, n, r, o, i)),
    (r = Zu()),
    e !== null && !Ne
      ? ((t.updateQueue = e.updateQueue),
        (t.flags &= -2053),
        (e.lanes &= ~i),
        Lt(e, t, i))
      : (ee && r && Ru(t), (t.flags |= 1), Pe(e, t, n, i), t.child)
  );
}
function Lc(e, t, n, r, i) {
  if (De(n)) {
    var o = !0;
    Co(t);
  } else o = !1;
  if ((nr(t, i), t.stateNode === null))
    ao(e, t), Nh(t, n, r), Hs(t, n, r, i), (r = !0);
  else if (e === null) {
    var l = t.stateNode,
      s = t.memoizedProps;
    l.props = s;
    var u = l.context,
      a = n.contextType;
    typeof a == "object" && a !== null
      ? (a = be(a))
      : ((a = De(n) ? xn : Ee.current), (a = lr(t, a)));
    var c = n.getDerivedStateFromProps,
      f =
        typeof c == "function" ||
        typeof l.getSnapshotBeforeUpdate == "function";
    f ||
      (typeof l.UNSAFE_componentWillReceiveProps != "function" &&
        typeof l.componentWillReceiveProps != "function") ||
      ((s !== r || u !== a) && Pc(t, l, r, a)),
      (Ot = !1);
    var d = t.memoizedState;
    (l.state = d),
      Lo(t, r, l, i),
      (u = t.memoizedState),
      s !== r || d !== u || Ae.current || Ot
        ? (typeof c == "function" && (Us(t, n, c, r), (u = t.memoizedState)),
          (s = Ot || Ec(t, n, s, r, d, u, a))
            ? (f ||
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
      ch(e, t),
      (s = t.memoizedProps),
      (a = t.type === t.elementType ? s : rt(t.type, s)),
      (l.props = a),
      (f = t.pendingProps),
      (d = l.context),
      (u = n.contextType),
      typeof u == "object" && u !== null
        ? (u = be(u))
        : ((u = De(n) ? xn : Ee.current), (u = lr(t, u)));
    var x = n.getDerivedStateFromProps;
    (c =
      typeof x == "function" ||
      typeof l.getSnapshotBeforeUpdate == "function") ||
      (typeof l.UNSAFE_componentWillReceiveProps != "function" &&
        typeof l.componentWillReceiveProps != "function") ||
      ((s !== f || d !== u) && Pc(t, l, r, u)),
      (Ot = !1),
      (d = t.memoizedState),
      (l.state = d),
      Lo(t, r, l, i);
    var g = t.memoizedState;
    s !== f || d !== g || Ae.current || Ot
      ? (typeof x == "function" && (Us(t, n, x, r), (g = t.memoizedState)),
        (a = Ot || Ec(t, n, a, r, d, g, u) || !1)
          ? (c ||
              (typeof l.UNSAFE_componentWillUpdate != "function" &&
                typeof l.componentWillUpdate != "function") ||
              (typeof l.componentWillUpdate == "function" &&
                l.componentWillUpdate(r, g, u),
              typeof l.UNSAFE_componentWillUpdate == "function" &&
                l.UNSAFE_componentWillUpdate(r, g, u)),
            typeof l.componentDidUpdate == "function" && (t.flags |= 4),
            typeof l.getSnapshotBeforeUpdate == "function" && (t.flags |= 1024))
          : (typeof l.componentDidUpdate != "function" ||
              (s === e.memoizedProps && d === e.memoizedState) ||
              (t.flags |= 4),
            typeof l.getSnapshotBeforeUpdate != "function" ||
              (s === e.memoizedProps && d === e.memoizedState) ||
              (t.flags |= 1024),
            (t.memoizedProps = r),
            (t.memoizedState = g)),
        (l.props = r),
        (l.state = g),
        (l.context = u),
        (r = a))
      : (typeof l.componentDidUpdate != "function" ||
          (s === e.memoizedProps && d === e.memoizedState) ||
          (t.flags |= 4),
        typeof l.getSnapshotBeforeUpdate != "function" ||
          (s === e.memoizedProps && d === e.memoizedState) ||
          (t.flags |= 1024),
        (r = !1));
  }
  return Bs(e, t, n, r, o, i);
}
function Bs(e, t, n, r, i, o) {
  Oh(e, t);
  var l = (t.flags & 128) !== 0;
  if (!r && !l) return i && mc(t, n, !1), Lt(e, t, o);
  (r = t.stateNode), (oy.current = t);
  var s =
    l && typeof n.getDerivedStateFromError != "function" ? null : r.render();
  return (
    (t.flags |= 1),
    e !== null && l
      ? ((t.child = ur(t, e.child, null, o)), (t.child = ur(t, null, s, o)))
      : Pe(e, t, s, o),
    (t.memoizedState = r.state),
    i && mc(t, n, !0),
    t.child
  );
}
function Rh(e) {
  var t = e.stateNode;
  t.pendingContext
    ? pc(e, t.pendingContext, t.pendingContext !== t.context)
    : t.context && pc(e, t.context, !1),
    Yu(e, t.containerInfo);
}
function Nc(e, t, n, r, i) {
  return sr(), Iu(i), (t.flags |= 256), Pe(e, t, n, r), t.child;
}
var Ys = { dehydrated: null, treeContext: null, retryLane: 0 };
function Gs(e) {
  return { baseLanes: e, cachePool: null, transitions: null };
}
function Fh(e, t, n) {
  var r = t.pendingProps,
    i = te.current,
    o = !1,
    l = (t.flags & 128) !== 0,
    s;
  if (
    ((s = l) ||
      (s = e !== null && e.memoizedState === null ? !1 : (i & 2) !== 0),
    s
      ? ((o = !0), (t.flags &= -129))
      : (e === null || e.memoizedState !== null) && (i |= 1),
    q(te, i & 1),
    e === null)
  )
    return (
      Fs(t),
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
                : (o = al(l, r, 0, null)),
              (e = pn(e, r, n, null)),
              (o.return = t),
              (e.return = t),
              (o.sibling = e),
              (t.child = o),
              (t.child.memoizedState = Gs(n)),
              (t.memoizedState = Ys),
              e)
            : bu(t, l))
    );
  if (((i = e.memoizedState), i !== null && ((s = i.dehydrated), s !== null)))
    return ly(e, t, l, r, s, i, n);
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
      s !== null ? (o = Kt(s, o)) : ((o = pn(o, l, n, null)), (o.flags |= 2)),
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
      (t.memoizedState = Ys),
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
function bu(e, t) {
  return (
    (t = al({ mode: "visible", children: t }, e.mode, 0, null)),
    (t.return = e),
    (e.child = t)
  );
}
function Qi(e, t, n, r) {
  return (
    r !== null && Iu(r),
    ur(t, e.child, null, n),
    (e = bu(t, t.pendingProps.children)),
    (e.flags |= 2),
    (t.memoizedState = null),
    e
  );
}
function ly(e, t, n, r, i, o, l) {
  if (n)
    return t.flags & 256
      ? ((t.flags &= -257), (r = Vl(Error(M(422)))), Qi(e, t, l, r))
      : t.memoizedState !== null
      ? ((t.child = e.child), (t.flags |= 128), null)
      : ((o = r.fallback),
        (i = t.mode),
        (r = al({ mode: "visible", children: r.children }, i, 0, null)),
        (o = pn(o, i, l, null)),
        (o.flags |= 2),
        (r.return = t),
        (o.return = t),
        (r.sibling = o),
        (t.child = r),
        t.mode & 1 && ur(t, e.child, null, l),
        (t.child.memoizedState = Gs(l)),
        (t.memoizedState = Ys),
        o);
  if (!(t.mode & 1)) return Qi(e, t, l, null);
  if (i.data === "$!") {
    if (((r = i.nextSibling && i.nextSibling.dataset), r)) var s = r.dgst;
    return (r = s), (o = Error(M(419))), (r = Vl(o, r, void 0)), Qi(e, t, l, r);
  }
  if (((s = (l & e.childLanes) !== 0), Ne || s)) {
    if (((r = pe), r !== null)) {
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
          ((o.retryLane = i), _t(e, i), at(r, e, i, -1));
    }
    return oa(), (r = Vl(Error(M(421)))), Qi(e, t, l, r);
  }
  return i.data === "$?"
    ? ((t.flags |= 128),
      (t.child = e.child),
      (t = vy.bind(null, e)),
      (i._reactRetry = t),
      null)
    : ((e = o.treeContext),
      (Ie = Yt(i.nextSibling)),
      (Ue = t),
      (ee = !0),
      (ot = null),
      e !== null &&
        ((Qe[Xe++] = St),
        (Qe[Xe++] = Et),
        (Qe[Xe++] = vn),
        (St = e.id),
        (Et = e.overflow),
        (vn = t)),
      (t = bu(t, r.children)),
      (t.flags |= 4096),
      t);
}
function $c(e, t, n) {
  e.lanes |= t;
  var r = e.alternate;
  r !== null && (r.lanes |= t), Is(e.return, t, n);
}
function Bl(e, t, n, r, i) {
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
function Ih(e, t, n) {
  var r = t.pendingProps,
    i = r.revealOrder,
    o = r.tail;
  if ((Pe(e, t, r.children, n), (r = te.current), r & 2))
    (r = (r & 1) | 2), (t.flags |= 128);
  else {
    if (e !== null && e.flags & 128)
      e: for (e = t.child; e !== null; ) {
        if (e.tag === 13) e.memoizedState !== null && $c(e, n, t);
        else if (e.tag === 19) $c(e, n, t);
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
  if ((q(te, r), !(t.mode & 1))) t.memoizedState = null;
  else
    switch (i) {
      case "forwards":
        for (n = t.child, i = null; n !== null; )
          (e = n.alternate),
            e !== null && No(e) === null && (i = n),
            (n = n.sibling);
        (n = i),
          n === null
            ? ((i = t.child), (t.child = null))
            : ((i = n.sibling), (n.sibling = null)),
          Bl(t, !1, i, n, o);
        break;
      case "backwards":
        for (n = null, i = t.child, t.child = null; i !== null; ) {
          if (((e = i.alternate), e !== null && No(e) === null)) {
            t.child = i;
            break;
          }
          (e = i.sibling), (i.sibling = n), (n = i), (i = e);
        }
        Bl(t, !0, n, null, o);
        break;
      case "together":
        Bl(t, !1, null, null, void 0);
        break;
      default:
        t.memoizedState = null;
    }
  return t.child;
}
function ao(e, t) {
  !(t.mode & 1) &&
    e !== null &&
    ((e.alternate = null), (t.alternate = null), (t.flags |= 2));
}
function Lt(e, t, n) {
  if (
    (e !== null && (t.dependencies = e.dependencies),
    (kn |= t.lanes),
    !(n & t.childLanes))
  )
    return null;
  if (e !== null && t.child !== e.child) throw Error(M(153));
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
function sy(e, t, n) {
  switch (t.tag) {
    case 3:
      Rh(t), sr();
      break;
    case 5:
      fh(t);
      break;
    case 1:
      De(t.type) && Co(t);
      break;
    case 4:
      Yu(t, t.stateNode.containerInfo);
      break;
    case 10:
      var r = t.type._context,
        i = t.memoizedProps.value;
      q(To, r._currentValue), (r._currentValue = i);
      break;
    case 13:
      if (((r = t.memoizedState), r !== null))
        return r.dehydrated !== null
          ? (q(te, te.current & 1), (t.flags |= 128), null)
          : n & t.child.childLanes
          ? Fh(e, t, n)
          : (q(te, te.current & 1),
            (e = Lt(e, t, n)),
            e !== null ? e.sibling : null);
      q(te, te.current & 1);
      break;
    case 19:
      if (((r = (n & t.childLanes) !== 0), e.flags & 128)) {
        if (r) return Ih(e, t, n);
        t.flags |= 128;
      }
      if (
        ((i = t.memoizedState),
        i !== null &&
          ((i.rendering = null), (i.tail = null), (i.lastEffect = null)),
        q(te, te.current),
        r)
      )
        break;
      return null;
    case 22:
    case 23:
      return (t.lanes = 0), zh(e, t, n);
  }
  return Lt(e, t, n);
}
var Uh, Qs, Hh, Wh;
Uh = function (e, t) {
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
Qs = function () {};
Hh = function (e, t, n, r) {
  var i = e.memoizedProps;
  if (i !== r) {
    (e = t.stateNode), fn(yt.current);
    var o = null;
    switch (n) {
      case "input":
        (i = ms(e, i)), (r = ms(e, r)), (o = []);
        break;
      case "select":
        (i = re({}, i, { value: void 0 })),
          (r = re({}, r, { value: void 0 })),
          (o = []);
        break;
      case "textarea":
        (i = xs(e, i)), (r = xs(e, r)), (o = []);
        break;
      default:
        typeof i.onClick != "function" &&
          typeof r.onClick == "function" &&
          (e.onclick = Eo);
    }
    ws(n, r);
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
            (ei.hasOwnProperty(a)
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
              (ei.hasOwnProperty(a)
                ? (u != null && a === "onScroll" && J("scroll", e),
                  o || s === u || (o = []))
                : (o = o || []).push(a, u));
    }
    n && (o = o || []).push("style", n);
    var a = o;
    (t.updateQueue = a) && (t.flags |= 4);
  }
};
Wh = function (e, t, n, r) {
  n !== r && (t.flags |= 4);
};
function _r(e, t) {
  if (!ee)
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
function we(e) {
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
function uy(e, t, n) {
  var r = t.pendingProps;
  switch ((Fu(t), t.tag)) {
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
      return we(t), null;
    case 1:
      return De(t.type) && Po(), we(t), null;
    case 3:
      return (
        (r = t.stateNode),
        ar(),
        b(Ae),
        b(Ee),
        Qu(),
        r.pendingContext &&
          ((r.context = r.pendingContext), (r.pendingContext = null)),
        (e === null || e.child === null) &&
          (Yi(t)
            ? (t.flags |= 4)
            : e === null ||
              (e.memoizedState.isDehydrated && !(t.flags & 256)) ||
              ((t.flags |= 1024), ot !== null && (tu(ot), (ot = null)))),
        Qs(e, t),
        we(t),
        null
      );
    case 5:
      Gu(t);
      var i = fn(di.current);
      if (((n = t.type), e !== null && t.stateNode != null))
        Hh(e, t, n, r, i),
          e.ref !== t.ref && ((t.flags |= 512), (t.flags |= 2097152));
      else {
        if (!r) {
          if (t.stateNode === null) throw Error(M(166));
          return we(t), null;
        }
        if (((e = fn(yt.current)), Yi(t))) {
          (r = t.stateNode), (n = t.type);
          var o = t.memoizedProps;
          switch (((r[pt] = t), (r[ci] = o), (e = (t.mode & 1) !== 0), n)) {
            case "dialog":
              J("cancel", r), J("close", r);
              break;
            case "iframe":
            case "object":
            case "embed":
              J("load", r);
              break;
            case "video":
            case "audio":
              for (i = 0; i < Ir.length; i++) J(Ir[i], r);
              break;
            case "source":
              J("error", r);
              break;
            case "img":
            case "image":
            case "link":
              J("error", r), J("load", r);
              break;
            case "details":
              J("toggle", r);
              break;
            case "input":
              Ha(r, o), J("invalid", r);
              break;
            case "select":
              (r._wrapperState = { wasMultiple: !!o.multiple }),
                J("invalid", r);
              break;
            case "textarea":
              Va(r, o), J("invalid", r);
          }
          ws(n, o), (i = null);
          for (var l in o)
            if (o.hasOwnProperty(l)) {
              var s = o[l];
              l === "children"
                ? typeof s == "string"
                  ? r.textContent !== s &&
                    (o.suppressHydrationWarning !== !0 &&
                      Bi(r.textContent, s, e),
                    (i = ["children", s]))
                  : typeof s == "number" &&
                    r.textContent !== "" + s &&
                    (o.suppressHydrationWarning !== !0 &&
                      Bi(r.textContent, s, e),
                    (i = ["children", "" + s]))
                : ei.hasOwnProperty(l) &&
                  s != null &&
                  l === "onScroll" &&
                  J("scroll", r);
            }
          switch (n) {
            case "input":
              Oi(r), Wa(r, o, !0);
              break;
            case "textarea":
              Oi(r), Ba(r);
              break;
            case "select":
            case "option":
              break;
            default:
              typeof o.onClick == "function" && (r.onclick = Eo);
          }
          (r = i), (t.updateQueue = r), r !== null && (t.flags |= 4);
        } else {
          (l = i.nodeType === 9 ? i : i.ownerDocument),
            e === "http://www.w3.org/1999/xhtml" && (e = yd(n)),
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
            (e[pt] = t),
            (e[ci] = r),
            Uh(e, t, !1, !1),
            (t.stateNode = e);
          e: {
            switch (((l = ks(n, r)), n)) {
              case "dialog":
                J("cancel", e), J("close", e), (i = r);
                break;
              case "iframe":
              case "object":
              case "embed":
                J("load", e), (i = r);
                break;
              case "video":
              case "audio":
                for (i = 0; i < Ir.length; i++) J(Ir[i], e);
                i = r;
                break;
              case "source":
                J("error", e), (i = r);
                break;
              case "img":
              case "image":
              case "link":
                J("error", e), J("load", e), (i = r);
                break;
              case "details":
                J("toggle", e), (i = r);
                break;
              case "input":
                Ha(e, r), (i = ms(e, r)), J("invalid", e);
                break;
              case "option":
                i = r;
                break;
              case "select":
                (e._wrapperState = { wasMultiple: !!r.multiple }),
                  (i = re({}, r, { value: void 0 })),
                  J("invalid", e);
                break;
              case "textarea":
                Va(e, r), (i = xs(e, r)), J("invalid", e);
                break;
              default:
                i = r;
            }
            ws(n, i), (s = i);
            for (o in s)
              if (s.hasOwnProperty(o)) {
                var u = s[o];
                o === "style"
                  ? vd(e, u)
                  : o === "dangerouslySetInnerHTML"
                  ? ((u = u ? u.__html : void 0), u != null && gd(e, u))
                  : o === "children"
                  ? typeof u == "string"
                    ? (n !== "textarea" || u !== "") && ti(e, u)
                    : typeof u == "number" && ti(e, "" + u)
                  : o !== "suppressContentEditableWarning" &&
                    o !== "suppressHydrationWarning" &&
                    o !== "autoFocus" &&
                    (ei.hasOwnProperty(o)
                      ? u != null && o === "onScroll" && J("scroll", e)
                      : u != null && Eu(e, o, u, l));
              }
            switch (n) {
              case "input":
                Oi(e), Wa(e, r, !1);
                break;
              case "textarea":
                Oi(e), Ba(e);
                break;
              case "option":
                r.value != null && e.setAttribute("value", "" + Zt(r.value));
                break;
              case "select":
                (e.multiple = !!r.multiple),
                  (o = r.value),
                  o != null
                    ? Jn(e, !!r.multiple, o, !1)
                    : r.defaultValue != null &&
                      Jn(e, !!r.multiple, r.defaultValue, !0);
                break;
              default:
                typeof i.onClick == "function" && (e.onclick = Eo);
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
      return we(t), null;
    case 6:
      if (e && t.stateNode != null) Wh(e, t, e.memoizedProps, r);
      else {
        if (typeof r != "string" && t.stateNode === null) throw Error(M(166));
        if (((n = fn(di.current)), fn(yt.current), Yi(t))) {
          if (
            ((r = t.stateNode),
            (n = t.memoizedProps),
            (r[pt] = t),
            (o = r.nodeValue !== n) && ((e = Ue), e !== null))
          )
            switch (e.tag) {
              case 3:
                Bi(r.nodeValue, n, (e.mode & 1) !== 0);
                break;
              case 5:
                e.memoizedProps.suppressHydrationWarning !== !0 &&
                  Bi(r.nodeValue, n, (e.mode & 1) !== 0);
            }
          o && (t.flags |= 4);
        } else
          (r = (n.nodeType === 9 ? n : n.ownerDocument).createTextNode(r)),
            (r[pt] = t),
            (t.stateNode = r);
      }
      return we(t), null;
    case 13:
      if (
        (b(te),
        (r = t.memoizedState),
        e === null ||
          (e.memoizedState !== null && e.memoizedState.dehydrated !== null))
      ) {
        if (ee && Ie !== null && t.mode & 1 && !(t.flags & 128))
          lh(), sr(), (t.flags |= 98560), (o = !1);
        else if (((o = Yi(t)), r !== null && r.dehydrated !== null)) {
          if (e === null) {
            if (!o) throw Error(M(318));
            if (
              ((o = t.memoizedState),
              (o = o !== null ? o.dehydrated : null),
              !o)
            )
              throw Error(M(317));
            o[pt] = t;
          } else
            sr(), !(t.flags & 128) && (t.memoizedState = null), (t.flags |= 4);
          we(t), (o = !1);
        } else ot !== null && (tu(ot), (ot = null)), (o = !0);
        if (!o) return t.flags & 65536 ? t : null;
      }
      return t.flags & 128
        ? ((t.lanes = n), t)
        : ((r = r !== null),
          r !== (e !== null && e.memoizedState !== null) &&
            r &&
            ((t.child.flags |= 8192),
            t.mode & 1 &&
              (e === null || te.current & 1 ? ce === 0 && (ce = 3) : oa())),
          t.updateQueue !== null && (t.flags |= 4),
          we(t),
          null);
    case 4:
      return (
        ar(), Qs(e, t), e === null && ui(t.stateNode.containerInfo), we(t), null
      );
    case 10:
      return Wu(t.type._context), we(t), null;
    case 17:
      return De(t.type) && Po(), we(t), null;
    case 19:
      if ((b(te), (o = t.memoizedState), o === null)) return we(t), null;
      if (((r = (t.flags & 128) !== 0), (l = o.rendering), l === null))
        if (r) _r(o, !1);
        else {
          if (ce !== 0 || (e !== null && e.flags & 128))
            for (e = t.child; e !== null; ) {
              if (((l = No(e)), l !== null)) {
                for (
                  t.flags |= 128,
                    _r(o, !1),
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
                return q(te, (te.current & 1) | 2), t.child;
              }
              e = e.sibling;
            }
          o.tail !== null &&
            le() > fr &&
            ((t.flags |= 128), (r = !0), _r(o, !1), (t.lanes = 4194304));
        }
      else {
        if (!r)
          if (((e = No(l)), e !== null)) {
            if (
              ((t.flags |= 128),
              (r = !0),
              (n = e.updateQueue),
              n !== null && ((t.updateQueue = n), (t.flags |= 4)),
              _r(o, !0),
              o.tail === null && o.tailMode === "hidden" && !l.alternate && !ee)
            )
              return we(t), null;
          } else
            2 * le() - o.renderingStartTime > fr &&
              n !== 1073741824 &&
              ((t.flags |= 128), (r = !0), _r(o, !1), (t.lanes = 4194304));
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
          (o.renderingStartTime = le()),
          (t.sibling = null),
          (n = te.current),
          q(te, r ? (n & 1) | 2 : n & 1),
          t)
        : (we(t), null);
    case 22:
    case 23:
      return (
        ia(),
        (r = t.memoizedState !== null),
        e !== null && (e.memoizedState !== null) !== r && (t.flags |= 8192),
        r && t.mode & 1
          ? Fe & 1073741824 && (we(t), t.subtreeFlags & 6 && (t.flags |= 8192))
          : we(t),
        null
      );
    case 24:
      return null;
    case 25:
      return null;
  }
  throw Error(M(156, t.tag));
}
function ay(e, t) {
  switch ((Fu(t), t.tag)) {
    case 1:
      return (
        De(t.type) && Po(),
        (e = t.flags),
        e & 65536 ? ((t.flags = (e & -65537) | 128), t) : null
      );
    case 3:
      return (
        ar(),
        b(Ae),
        b(Ee),
        Qu(),
        (e = t.flags),
        e & 65536 && !(e & 128) ? ((t.flags = (e & -65537) | 128), t) : null
      );
    case 5:
      return Gu(t), null;
    case 13:
      if ((b(te), (e = t.memoizedState), e !== null && e.dehydrated !== null)) {
        if (t.alternate === null) throw Error(M(340));
        sr();
      }
      return (
        (e = t.flags), e & 65536 ? ((t.flags = (e & -65537) | 128), t) : null
      );
    case 19:
      return b(te), null;
    case 4:
      return ar(), null;
    case 10:
      return Wu(t.type._context), null;
    case 22:
    case 23:
      return ia(), null;
    case 24:
      return null;
    default:
      return null;
  }
}
var Xi = !1,
  ke = !1,
  cy = typeof WeakSet == "function" ? WeakSet : Set,
  $ = null;
function Qn(e, t) {
  var n = e.ref;
  if (n !== null)
    if (typeof n == "function")
      try {
        n(null);
      } catch (r) {
        ie(e, t, r);
      }
    else n.current = null;
}
function Xs(e, t, n) {
  try {
    n();
  } catch (r) {
    ie(e, t, r);
  }
}
var Ac = !1;
function fy(e, t) {
  if (((Ns = wo), (e = Qd()), Ou(e))) {
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
            c = 0,
            f = e,
            d = null;
          t: for (;;) {
            for (
              var x;
              f !== n || (i !== 0 && f.nodeType !== 3) || (s = l + i),
                f !== o || (r !== 0 && f.nodeType !== 3) || (u = l + r),
                f.nodeType === 3 && (l += f.nodeValue.length),
                (x = f.firstChild) !== null;

            )
              (d = f), (f = x);
            for (;;) {
              if (f === e) break t;
              if (
                (d === n && ++a === i && (s = l),
                d === o && ++c === r && (u = l),
                (x = f.nextSibling) !== null)
              )
                break;
              (f = d), (d = f.parentNode);
            }
            f = x;
          }
          n = s === -1 || u === -1 ? null : { start: s, end: u };
        } else n = null;
      }
    n = n || { start: 0, end: 0 };
  } else n = null;
  for ($s = { focusedElem: e, selectionRange: n }, wo = !1, $ = t; $ !== null; )
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
                    E = g.memoizedState,
                    m = t.stateNode,
                    p = m.getSnapshotBeforeUpdate(
                      t.elementType === t.type ? v : rt(t.type, v),
                      E
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
                throw Error(M(163));
            }
        } catch (w) {
          ie(t, t.return, w);
        }
        if (((e = t.sibling), e !== null)) {
          (e.return = t.return), ($ = e);
          break;
        }
        $ = t.return;
      }
  return (g = Ac), (Ac = !1), g;
}
function Zr(e, t, n) {
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
function sl(e, t) {
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
function Vh(e) {
  var t = e.alternate;
  t !== null && ((e.alternate = null), Vh(t)),
    (e.child = null),
    (e.deletions = null),
    (e.sibling = null),
    e.tag === 5 &&
      ((t = e.stateNode),
      t !== null &&
        (delete t[pt], delete t[ci], delete t[zs], delete t[Qm], delete t[Xm])),
    (e.stateNode = null),
    (e.return = null),
    (e.dependencies = null),
    (e.memoizedProps = null),
    (e.memoizedState = null),
    (e.pendingProps = null),
    (e.stateNode = null),
    (e.updateQueue = null);
}
function Bh(e) {
  return e.tag === 5 || e.tag === 3 || e.tag === 4;
}
function Dc(e) {
  e: for (;;) {
    for (; e.sibling === null; ) {
      if (e.return === null || Bh(e.return)) return null;
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
          n != null || t.onclick !== null || (t.onclick = Eo));
  else if (r !== 4 && ((e = e.child), e !== null))
    for (Zs(e, t, n), e = e.sibling; e !== null; ) Zs(e, t, n), (e = e.sibling);
}
function qs(e, t, n) {
  var r = e.tag;
  if (r === 5 || r === 6)
    (e = e.stateNode), t ? n.insertBefore(e, t) : n.appendChild(e);
  else if (r !== 4 && ((e = e.child), e !== null))
    for (qs(e, t, n), e = e.sibling; e !== null; ) qs(e, t, n), (e = e.sibling);
}
var ye = null,
  it = !1;
function Dt(e, t, n) {
  for (n = n.child; n !== null; ) Yh(e, t, n), (n = n.sibling);
}
function Yh(e, t, n) {
  if (mt && typeof mt.onCommitFiberUnmount == "function")
    try {
      mt.onCommitFiberUnmount(bo, n);
    } catch {}
  switch (n.tag) {
    case 5:
      ke || Qn(n, t);
    case 6:
      var r = ye,
        i = it;
      (ye = null),
        Dt(e, t, n),
        (ye = r),
        (it = i),
        ye !== null &&
          (it
            ? ((e = ye),
              (n = n.stateNode),
              e.nodeType === 8 ? e.parentNode.removeChild(n) : e.removeChild(n))
            : ye.removeChild(n.stateNode));
      break;
    case 18:
      ye !== null &&
        (it
          ? ((e = ye),
            (n = n.stateNode),
            e.nodeType === 8
              ? Rl(e.parentNode, n)
              : e.nodeType === 1 && Rl(e, n),
            oi(e))
          : Rl(ye, n.stateNode));
      break;
    case 4:
      (r = ye),
        (i = it),
        (ye = n.stateNode.containerInfo),
        (it = !0),
        Dt(e, t, n),
        (ye = r),
        (it = i);
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
      Dt(e, t, n);
      break;
    case 1:
      if (
        !ke &&
        (Qn(n, t),
        (r = n.stateNode),
        typeof r.componentWillUnmount == "function")
      )
        try {
          (r.props = n.memoizedProps),
            (r.state = n.memoizedState),
            r.componentWillUnmount();
        } catch (s) {
          ie(n, t, s);
        }
      Dt(e, t, n);
      break;
    case 21:
      Dt(e, t, n);
      break;
    case 22:
      n.mode & 1
        ? ((ke = (r = ke) || n.memoizedState !== null), Dt(e, t, n), (ke = r))
        : Dt(e, t, n);
      break;
    default:
      Dt(e, t, n);
  }
}
function zc(e) {
  var t = e.updateQueue;
  if (t !== null) {
    e.updateQueue = null;
    var n = e.stateNode;
    n === null && (n = e.stateNode = new cy()),
      t.forEach(function (r) {
        var i = wy.bind(null, e, r);
        n.has(r) || (n.add(r), r.then(i, i));
      });
  }
}
function nt(e, t) {
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
              (ye = s.stateNode), (it = !1);
              break e;
            case 3:
              (ye = s.stateNode.containerInfo), (it = !0);
              break e;
            case 4:
              (ye = s.stateNode.containerInfo), (it = !0);
              break e;
          }
          s = s.return;
        }
        if (ye === null) throw Error(M(160));
        Yh(o, l, i), (ye = null), (it = !1);
        var u = i.alternate;
        u !== null && (u.return = null), (i.return = null);
      } catch (a) {
        ie(i, t, a);
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
      if ((nt(t, e), ft(e), r & 4)) {
        try {
          Zr(3, e, e.return), sl(3, e);
        } catch (v) {
          ie(e, e.return, v);
        }
        try {
          Zr(5, e, e.return);
        } catch (v) {
          ie(e, e.return, v);
        }
      }
      break;
    case 1:
      nt(t, e), ft(e), r & 512 && n !== null && Qn(n, n.return);
      break;
    case 5:
      if (
        (nt(t, e),
        ft(e),
        r & 512 && n !== null && Qn(n, n.return),
        e.flags & 32)
      ) {
        var i = e.stateNode;
        try {
          ti(i, "");
        } catch (v) {
          ie(e, e.return, v);
        }
      }
      if (r & 4 && ((i = e.stateNode), i != null)) {
        var o = e.memoizedProps,
          l = n !== null ? n.memoizedProps : o,
          s = e.type,
          u = e.updateQueue;
        if (((e.updateQueue = null), u !== null))
          try {
            s === "input" && o.type === "radio" && o.name != null && pd(i, o),
              ks(s, l);
            var a = ks(s, o);
            for (l = 0; l < u.length; l += 2) {
              var c = u[l],
                f = u[l + 1];
              c === "style"
                ? vd(i, f)
                : c === "dangerouslySetInnerHTML"
                ? gd(i, f)
                : c === "children"
                ? ti(i, f)
                : Eu(i, c, f, a);
            }
            switch (s) {
              case "input":
                ys(i, o);
                break;
              case "textarea":
                md(i, o);
                break;
              case "select":
                var d = i._wrapperState.wasMultiple;
                i._wrapperState.wasMultiple = !!o.multiple;
                var x = o.value;
                x != null
                  ? Jn(i, !!o.multiple, x, !1)
                  : d !== !!o.multiple &&
                    (o.defaultValue != null
                      ? Jn(i, !!o.multiple, o.defaultValue, !0)
                      : Jn(i, !!o.multiple, o.multiple ? [] : "", !1));
            }
            i[ci] = o;
          } catch (v) {
            ie(e, e.return, v);
          }
      }
      break;
    case 6:
      if ((nt(t, e), ft(e), r & 4)) {
        if (e.stateNode === null) throw Error(M(162));
        (i = e.stateNode), (o = e.memoizedProps);
        try {
          i.nodeValue = o;
        } catch (v) {
          ie(e, e.return, v);
        }
      }
      break;
    case 3:
      if (
        (nt(t, e), ft(e), r & 4 && n !== null && n.memoizedState.isDehydrated)
      )
        try {
          oi(t.containerInfo);
        } catch (v) {
          ie(e, e.return, v);
        }
      break;
    case 4:
      nt(t, e), ft(e);
      break;
    case 13:
      nt(t, e),
        ft(e),
        (i = e.child),
        i.flags & 8192 &&
          ((o = i.memoizedState !== null),
          (i.stateNode.isHidden = o),
          !o ||
            (i.alternate !== null && i.alternate.memoizedState !== null) ||
            (na = le())),
        r & 4 && zc(e);
      break;
    case 22:
      if (
        ((c = n !== null && n.memoizedState !== null),
        e.mode & 1 ? ((ke = (a = ke) || c), nt(t, e), (ke = a)) : nt(t, e),
        ft(e),
        r & 8192)
      ) {
        if (
          ((a = e.memoizedState !== null),
          (e.stateNode.isHidden = a) && !c && e.mode & 1)
        )
          for ($ = e, c = e.child; c !== null; ) {
            for (f = $ = c; $ !== null; ) {
              switch (((d = $), (x = d.child), d.tag)) {
                case 0:
                case 11:
                case 14:
                case 15:
                  Zr(4, d, d.return);
                  break;
                case 1:
                  Qn(d, d.return);
                  var g = d.stateNode;
                  if (typeof g.componentWillUnmount == "function") {
                    (r = d), (n = d.return);
                    try {
                      (t = r),
                        (g.props = t.memoizedProps),
                        (g.state = t.memoizedState),
                        g.componentWillUnmount();
                    } catch (v) {
                      ie(r, n, v);
                    }
                  }
                  break;
                case 5:
                  Qn(d, d.return);
                  break;
                case 22:
                  if (d.memoizedState !== null) {
                    Rc(f);
                    continue;
                  }
              }
              x !== null ? ((x.return = d), ($ = x)) : Rc(f);
            }
            c = c.sibling;
          }
        e: for (c = null, f = e; ; ) {
          if (f.tag === 5) {
            if (c === null) {
              c = f;
              try {
                (i = f.stateNode),
                  a
                    ? ((o = i.style),
                      typeof o.setProperty == "function"
                        ? o.setProperty("display", "none", "important")
                        : (o.display = "none"))
                    : ((s = f.stateNode),
                      (u = f.memoizedProps.style),
                      (l =
                        u != null && u.hasOwnProperty("display")
                          ? u.display
                          : null),
                      (s.style.display = xd("display", l)));
              } catch (v) {
                ie(e, e.return, v);
              }
            }
          } else if (f.tag === 6) {
            if (c === null)
              try {
                f.stateNode.nodeValue = a ? "" : f.memoizedProps;
              } catch (v) {
                ie(e, e.return, v);
              }
          } else if (
            ((f.tag !== 22 && f.tag !== 23) ||
              f.memoizedState === null ||
              f === e) &&
            f.child !== null
          ) {
            (f.child.return = f), (f = f.child);
            continue;
          }
          if (f === e) break e;
          for (; f.sibling === null; ) {
            if (f.return === null || f.return === e) break e;
            c === f && (c = null), (f = f.return);
          }
          c === f && (c = null), (f.sibling.return = f.return), (f = f.sibling);
        }
      }
      break;
    case 19:
      nt(t, e), ft(e), r & 4 && zc(e);
      break;
    case 21:
      break;
    default:
      nt(t, e), ft(e);
  }
}
function ft(e) {
  var t = e.flags;
  if (t & 2) {
    try {
      e: {
        for (var n = e.return; n !== null; ) {
          if (Bh(n)) {
            var r = n;
            break e;
          }
          n = n.return;
        }
        throw Error(M(160));
      }
      switch (r.tag) {
        case 5:
          var i = r.stateNode;
          r.flags & 32 && (ti(i, ""), (r.flags &= -33));
          var o = Dc(e);
          qs(e, o, i);
          break;
        case 3:
        case 4:
          var l = r.stateNode.containerInfo,
            s = Dc(e);
          Zs(e, s, l);
          break;
        default:
          throw Error(M(161));
      }
    } catch (u) {
      ie(e, e.return, u);
    }
    e.flags &= -3;
  }
  t & 4096 && (e.flags &= -4097);
}
function dy(e, t, n) {
  ($ = e), Qh(e);
}
function Qh(e, t, n) {
  for (var r = (e.mode & 1) !== 0; $ !== null; ) {
    var i = $,
      o = i.child;
    if (i.tag === 22 && r) {
      var l = i.memoizedState !== null || Xi;
      if (!l) {
        var s = i.alternate,
          u = (s !== null && s.memoizedState !== null) || ke;
        s = Xi;
        var a = ke;
        if (((Xi = l), (ke = u) && !a))
          for ($ = i; $ !== null; )
            (l = $),
              (u = l.child),
              l.tag === 22 && l.memoizedState !== null
                ? Fc(i)
                : u !== null
                ? ((u.return = l), ($ = u))
                : Fc(i);
        for (; o !== null; ) ($ = o), Qh(o), (o = o.sibling);
        ($ = i), (Xi = s), (ke = a);
      }
      Oc(e);
    } else
      i.subtreeFlags & 8772 && o !== null ? ((o.return = i), ($ = o)) : Oc(e);
  }
}
function Oc(e) {
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
              ke || sl(5, t);
              break;
            case 1:
              var r = t.stateNode;
              if (t.flags & 4 && !ke)
                if (n === null) r.componentDidMount();
                else {
                  var i =
                    t.elementType === t.type
                      ? n.memoizedProps
                      : rt(t.type, n.memoizedProps);
                  r.componentDidUpdate(
                    i,
                    n.memoizedState,
                    r.__reactInternalSnapshotBeforeUpdate
                  );
                }
              var o = t.updateQueue;
              o !== null && wc(t, o, r);
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
                wc(t, l, n);
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
                  var c = a.memoizedState;
                  if (c !== null) {
                    var f = c.dehydrated;
                    f !== null && oi(f);
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
              throw Error(M(163));
          }
        ke || (t.flags & 512 && Ks(t));
      } catch (d) {
        ie(t, t.return, d);
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
function Rc(e) {
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
function Fc(e) {
  for (; $ !== null; ) {
    var t = $;
    try {
      switch (t.tag) {
        case 0:
        case 11:
        case 15:
          var n = t.return;
          try {
            sl(4, t);
          } catch (u) {
            ie(t, n, u);
          }
          break;
        case 1:
          var r = t.stateNode;
          if (typeof r.componentDidMount == "function") {
            var i = t.return;
            try {
              r.componentDidMount();
            } catch (u) {
              ie(t, i, u);
            }
          }
          var o = t.return;
          try {
            Ks(t);
          } catch (u) {
            ie(t, o, u);
          }
          break;
        case 5:
          var l = t.return;
          try {
            Ks(t);
          } catch (u) {
            ie(t, l, u);
          }
      }
    } catch (u) {
      ie(t, t.return, u);
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
var hy = Math.ceil,
  Do = At.ReactCurrentDispatcher,
  ea = At.ReactCurrentOwner,
  Je = At.ReactCurrentBatchConfig,
  W = 0,
  pe = null,
  ue = null,
  ge = 0,
  Fe = 0,
  Xn = bt(0),
  ce = 0,
  yi = null,
  kn = 0,
  ul = 0,
  ta = 0,
  qr = null,
  Le = null,
  na = 0,
  fr = 1 / 0,
  wt = null,
  zo = !1,
  Js = null,
  Qt = null,
  Ki = !1,
  Ht = null,
  Oo = 0,
  Jr = 0,
  bs = null,
  co = -1,
  fo = 0;
function Ce() {
  return W & 6 ? le() : co !== -1 ? co : (co = le());
}
function Xt(e) {
  return e.mode & 1
    ? W & 2 && ge !== 0
      ? ge & -ge
      : Zm.transition !== null
      ? (fo === 0 && (fo = Nd()), fo)
      : ((e = G),
        e !== 0 || ((e = window.event), (e = e === void 0 ? 16 : Fd(e.type))),
        e)
    : 1;
}
function at(e, t, n, r) {
  if (50 < Jr) throw ((Jr = 0), (bs = null), Error(M(185)));
  Ci(e, n, r),
    (!(W & 2) || e !== pe) &&
      (e === pe && (!(W & 2) && (ul |= n), ce === 4 && It(e, ge)),
      ze(e, r),
      n === 1 && W === 0 && !(t.mode & 1) && ((fr = le() + 500), il && en()));
}
function ze(e, t) {
  var n = e.callbackNode;
  Z0(e, t);
  var r = vo(e, e === pe ? ge : 0);
  if (r === 0)
    n !== null && Qa(n), (e.callbackNode = null), (e.callbackPriority = 0);
  else if (((t = r & -r), e.callbackPriority !== t)) {
    if ((n != null && Qa(n), t === 1))
      e.tag === 0 ? Km(Ic.bind(null, e)) : rh(Ic.bind(null, e)),
        Ym(function () {
          !(W & 6) && en();
        }),
        (n = null);
    else {
      switch ($d(r)) {
        case 1:
          n = Tu;
          break;
        case 4:
          n = _d;
          break;
        case 16:
          n = xo;
          break;
        case 536870912:
          n = Ld;
          break;
        default:
          n = xo;
      }
      n = tp(n, Xh.bind(null, e));
    }
    (e.callbackPriority = t), (e.callbackNode = n);
  }
}
function Xh(e, t) {
  if (((co = -1), (fo = 0), W & 6)) throw Error(M(327));
  var n = e.callbackNode;
  if (rr() && e.callbackNode !== n) return null;
  var r = vo(e, e === pe ? ge : 0);
  if (r === 0) return null;
  if (r & 30 || r & e.expiredLanes || t) t = Ro(e, r);
  else {
    t = r;
    var i = W;
    W |= 2;
    var o = Zh();
    (pe !== e || ge !== t) && ((wt = null), (fr = le() + 500), hn(e, t));
    do
      try {
        yy();
        break;
      } catch (s) {
        Kh(e, s);
      }
    while (!0);
    Hu(),
      (Do.current = o),
      (W = i),
      ue !== null ? (t = 0) : ((pe = null), (ge = 0), (t = ce));
  }
  if (t !== 0) {
    if (
      (t === 2 && ((i = Ms(e)), i !== 0 && ((r = i), (t = eu(e, i)))), t === 1)
    )
      throw ((n = yi), hn(e, 0), It(e, r), ze(e, le()), n);
    if (t === 6) It(e, r);
    else {
      if (
        ((i = e.current.alternate),
        !(r & 30) &&
          !py(i) &&
          ((t = Ro(e, r)),
          t === 2 && ((o = Ms(e)), o !== 0 && ((r = o), (t = eu(e, o)))),
          t === 1))
      )
        throw ((n = yi), hn(e, 0), It(e, r), ze(e, le()), n);
      switch (((e.finishedWork = i), (e.finishedLanes = r), t)) {
        case 0:
        case 1:
          throw Error(M(345));
        case 2:
          on(e, Le, wt);
          break;
        case 3:
          if (
            (It(e, r), (r & 130023424) === r && ((t = na + 500 - le()), 10 < t))
          ) {
            if (vo(e, 0) !== 0) break;
            if (((i = e.suspendedLanes), (i & r) !== r)) {
              Ce(), (e.pingedLanes |= e.suspendedLanes & i);
              break;
            }
            e.timeoutHandle = Ds(on.bind(null, e, Le, wt), t);
            break;
          }
          on(e, Le, wt);
          break;
        case 4:
          if ((It(e, r), (r & 4194240) === r)) break;
          for (t = e.eventTimes, i = -1; 0 < r; ) {
            var l = 31 - ut(r);
            (o = 1 << l), (l = t[l]), l > i && (i = l), (r &= ~o);
          }
          if (
            ((r = i),
            (r = le() - r),
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
                : 1960 * hy(r / 1960)) - r),
            10 < r)
          ) {
            e.timeoutHandle = Ds(on.bind(null, e, Le, wt), r);
            break;
          }
          on(e, Le, wt);
          break;
        case 5:
          on(e, Le, wt);
          break;
        default:
          throw Error(M(329));
      }
    }
  }
  return ze(e, le()), e.callbackNode === n ? Xh.bind(null, e) : null;
}
function eu(e, t) {
  var n = qr;
  return (
    e.current.memoizedState.isDehydrated && (hn(e, t).flags |= 256),
    (e = Ro(e, t)),
    e !== 2 && ((t = Le), (Le = n), t !== null && tu(t)),
    e
  );
}
function tu(e) {
  Le === null ? (Le = e) : Le.push.apply(Le, e);
}
function py(e) {
  for (var t = e; ; ) {
    if (t.flags & 16384) {
      var n = t.updateQueue;
      if (n !== null && ((n = n.stores), n !== null))
        for (var r = 0; r < n.length; r++) {
          var i = n[r],
            o = i.getSnapshot;
          i = i.value;
          try {
            if (!ct(o(), i)) return !1;
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
function It(e, t) {
  for (
    t &= ~ta,
      t &= ~ul,
      e.suspendedLanes |= t,
      e.pingedLanes &= ~t,
      e = e.expirationTimes;
    0 < t;

  ) {
    var n = 31 - ut(t),
      r = 1 << n;
    (e[n] = -1), (t &= ~r);
  }
}
function Ic(e) {
  if (W & 6) throw Error(M(327));
  rr();
  var t = vo(e, 0);
  if (!(t & 1)) return ze(e, le()), null;
  var n = Ro(e, t);
  if (e.tag !== 0 && n === 2) {
    var r = Ms(e);
    r !== 0 && ((t = r), (n = eu(e, r)));
  }
  if (n === 1) throw ((n = yi), hn(e, 0), It(e, t), ze(e, le()), n);
  if (n === 6) throw Error(M(345));
  return (
    (e.finishedWork = e.current.alternate),
    (e.finishedLanes = t),
    on(e, Le, wt),
    ze(e, le()),
    null
  );
}
function ra(e, t) {
  var n = W;
  W |= 1;
  try {
    return e(t);
  } finally {
    (W = n), W === 0 && ((fr = le() + 500), il && en());
  }
}
function Sn(e) {
  Ht !== null && Ht.tag === 0 && !(W & 6) && rr();
  var t = W;
  W |= 1;
  var n = Je.transition,
    r = G;
  try {
    if (((Je.transition = null), (G = 1), e)) return e();
  } finally {
    (G = r), (Je.transition = n), (W = t), !(W & 6) && en();
  }
}
function ia() {
  (Fe = Xn.current), b(Xn);
}
function hn(e, t) {
  (e.finishedWork = null), (e.finishedLanes = 0);
  var n = e.timeoutHandle;
  if ((n !== -1 && ((e.timeoutHandle = -1), Bm(n)), ue !== null))
    for (n = ue.return; n !== null; ) {
      var r = n;
      switch ((Fu(r), r.tag)) {
        case 1:
          (r = r.type.childContextTypes), r != null && Po();
          break;
        case 3:
          ar(), b(Ae), b(Ee), Qu();
          break;
        case 5:
          Gu(r);
          break;
        case 4:
          ar();
          break;
        case 13:
          b(te);
          break;
        case 19:
          b(te);
          break;
        case 10:
          Wu(r.type._context);
          break;
        case 22:
        case 23:
          ia();
      }
      n = n.return;
    }
  if (
    ((pe = e),
    (ue = e = Kt(e.current, null)),
    (ge = Fe = t),
    (ce = 0),
    (yi = null),
    (ta = ul = kn = 0),
    (Le = qr = null),
    cn !== null)
  ) {
    for (t = 0; t < cn.length; t++)
      if (((n = cn[t]), (r = n.interleaved), r !== null)) {
        n.interleaved = null;
        var i = r.next,
          o = n.pending;
        if (o !== null) {
          var l = o.next;
          (o.next = i), (r.next = l);
        }
        n.pending = r;
      }
    cn = null;
  }
  return e;
}
function Kh(e, t) {
  do {
    var n = ue;
    try {
      if ((Hu(), (so.current = Ao), $o)) {
        for (var r = ne.memoizedState; r !== null; ) {
          var i = r.queue;
          i !== null && (i.pending = null), (r = r.next);
        }
        $o = !1;
      }
      if (
        ((wn = 0),
        (he = ae = ne = null),
        (Kr = !1),
        (hi = 0),
        (ea.current = null),
        n === null || n.return === null)
      ) {
        (ce = 1), (yi = t), (ue = null);
        break;
      }
      e: {
        var o = e,
          l = n.return,
          s = n,
          u = t;
        if (
          ((t = ge),
          (s.flags |= 32768),
          u !== null && typeof u == "object" && typeof u.then == "function")
        ) {
          var a = u,
            c = s,
            f = c.tag;
          if (!(c.mode & 1) && (f === 0 || f === 11 || f === 15)) {
            var d = c.alternate;
            d
              ? ((c.updateQueue = d.updateQueue),
                (c.memoizedState = d.memoizedState),
                (c.lanes = d.lanes))
              : ((c.updateQueue = null), (c.memoizedState = null));
          }
          var x = Mc(l);
          if (x !== null) {
            (x.flags &= -257),
              jc(x, l, s, o, t),
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
              Cc(o, a, t), oa();
              break e;
            }
            u = Error(M(426));
          }
        } else if (ee && s.mode & 1) {
          var E = Mc(l);
          if (E !== null) {
            !(E.flags & 65536) && (E.flags |= 256),
              jc(E, l, s, o, t),
              Iu(cr(u, s));
            break e;
          }
        }
        (o = u = cr(u, s)),
          ce !== 4 && (ce = 2),
          qr === null ? (qr = [o]) : qr.push(o),
          (o = l);
        do {
          switch (o.tag) {
            case 3:
              (o.flags |= 65536), (t &= -t), (o.lanes |= t);
              var m = $h(o, u, t);
              vc(o, m);
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
                    (Qt === null || !Qt.has(y))))
              ) {
                (o.flags |= 65536), (t &= -t), (o.lanes |= t);
                var w = Ah(o, s, t);
                vc(o, w);
                break e;
              }
          }
          o = o.return;
        } while (o !== null);
      }
      Jh(n);
    } catch (k) {
      (t = k), ue === n && n !== null && (ue = n = n.return);
      continue;
    }
    break;
  } while (!0);
}
function Zh() {
  var e = Do.current;
  return (Do.current = Ao), e === null ? Ao : e;
}
function oa() {
  (ce === 0 || ce === 3 || ce === 2) && (ce = 4),
    pe === null || (!(kn & 268435455) && !(ul & 268435455)) || It(pe, ge);
}
function Ro(e, t) {
  var n = W;
  W |= 2;
  var r = Zh();
  (pe !== e || ge !== t) && ((wt = null), hn(e, t));
  do
    try {
      my();
      break;
    } catch (i) {
      Kh(e, i);
    }
  while (!0);
  if ((Hu(), (W = n), (Do.current = r), ue !== null)) throw Error(M(261));
  return (pe = null), (ge = 0), ce;
}
function my() {
  for (; ue !== null; ) qh(ue);
}
function yy() {
  for (; ue !== null && !H0(); ) qh(ue);
}
function qh(e) {
  var t = ep(e.alternate, e, Fe);
  (e.memoizedProps = e.pendingProps),
    t === null ? Jh(e) : (ue = t),
    (ea.current = null);
}
function Jh(e) {
  var t = e;
  do {
    var n = t.alternate;
    if (((e = t.return), t.flags & 32768)) {
      if (((n = ay(n, t)), n !== null)) {
        (n.flags &= 32767), (ue = n);
        return;
      }
      if (e !== null)
        (e.flags |= 32768), (e.subtreeFlags = 0), (e.deletions = null);
      else {
        (ce = 6), (ue = null);
        return;
      }
    } else if (((n = uy(n, t, Fe)), n !== null)) {
      ue = n;
      return;
    }
    if (((t = t.sibling), t !== null)) {
      ue = t;
      return;
    }
    ue = t = e;
  } while (t !== null);
  ce === 0 && (ce = 5);
}
function on(e, t, n) {
  var r = G,
    i = Je.transition;
  try {
    (Je.transition = null), (G = 1), gy(e, t, n, r);
  } finally {
    (Je.transition = i), (G = r);
  }
  return null;
}
function gy(e, t, n, r) {
  do rr();
  while (Ht !== null);
  if (W & 6) throw Error(M(327));
  n = e.finishedWork;
  var i = e.finishedLanes;
  if (n === null) return null;
  if (((e.finishedWork = null), (e.finishedLanes = 0), n === e.current))
    throw Error(M(177));
  (e.callbackNode = null), (e.callbackPriority = 0);
  var o = n.lanes | n.childLanes;
  if (
    (q0(e, o),
    e === pe && ((ue = pe = null), (ge = 0)),
    (!(n.subtreeFlags & 2064) && !(n.flags & 2064)) ||
      Ki ||
      ((Ki = !0),
      tp(xo, function () {
        return rr(), null;
      })),
    (o = (n.flags & 15990) !== 0),
    n.subtreeFlags & 15990 || o)
  ) {
    (o = Je.transition), (Je.transition = null);
    var l = G;
    G = 1;
    var s = W;
    (W |= 4),
      (ea.current = null),
      fy(e, n),
      Gh(n, e),
      Rm($s),
      (wo = !!Ns),
      ($s = Ns = null),
      (e.current = n),
      dy(n),
      W0(),
      (W = s),
      (G = l),
      (Je.transition = o);
  } else e.current = n;
  if (
    (Ki && ((Ki = !1), (Ht = e), (Oo = i)),
    (o = e.pendingLanes),
    o === 0 && (Qt = null),
    Y0(n.stateNode),
    ze(e, le()),
    t !== null)
  )
    for (r = e.onRecoverableError, n = 0; n < t.length; n++)
      (i = t[n]), r(i.value, { componentStack: i.stack, digest: i.digest });
  if (zo) throw ((zo = !1), (e = Js), (Js = null), e);
  return (
    Oo & 1 && e.tag !== 0 && rr(),
    (o = e.pendingLanes),
    o & 1 ? (e === bs ? Jr++ : ((Jr = 0), (bs = e))) : (Jr = 0),
    en(),
    null
  );
}
function rr() {
  if (Ht !== null) {
    var e = $d(Oo),
      t = Je.transition,
      n = G;
    try {
      if (((Je.transition = null), (G = 16 > e ? 16 : e), Ht === null))
        var r = !1;
      else {
        if (((e = Ht), (Ht = null), (Oo = 0), W & 6)) throw Error(M(331));
        var i = W;
        for (W |= 4, $ = e.current; $ !== null; ) {
          var o = $,
            l = o.child;
          if ($.flags & 16) {
            var s = o.deletions;
            if (s !== null) {
              for (var u = 0; u < s.length; u++) {
                var a = s[u];
                for ($ = a; $ !== null; ) {
                  var c = $;
                  switch (c.tag) {
                    case 0:
                    case 11:
                    case 15:
                      Zr(8, c, o);
                  }
                  var f = c.child;
                  if (f !== null) (f.return = c), ($ = f);
                  else
                    for (; $ !== null; ) {
                      c = $;
                      var d = c.sibling,
                        x = c.return;
                      if ((Vh(c), c === a)) {
                        $ = null;
                        break;
                      }
                      if (d !== null) {
                        (d.return = x), ($ = d);
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
                    var E = v.sibling;
                    (v.sibling = null), (v = E);
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
                    Zr(9, o, o.return);
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
                      sl(9, s);
                  }
                } catch (k) {
                  ie(s, s.return, k);
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
          ((W = i), en(), mt && typeof mt.onPostCommitFiberRoot == "function")
        )
          try {
            mt.onPostCommitFiberRoot(bo, e);
          } catch {}
        r = !0;
      }
      return r;
    } finally {
      (G = n), (Je.transition = t);
    }
  }
  return !1;
}
function Uc(e, t, n) {
  (t = cr(n, t)),
    (t = $h(e, t, 1)),
    (e = Gt(e, t, 1)),
    (t = Ce()),
    e !== null && (Ci(e, 1, t), ze(e, t));
}
function ie(e, t, n) {
  if (e.tag === 3) Uc(e, e, n);
  else
    for (; t !== null; ) {
      if (t.tag === 3) {
        Uc(t, e, n);
        break;
      } else if (t.tag === 1) {
        var r = t.stateNode;
        if (
          typeof t.type.getDerivedStateFromError == "function" ||
          (typeof r.componentDidCatch == "function" &&
            (Qt === null || !Qt.has(r)))
        ) {
          (e = cr(n, e)),
            (e = Ah(t, e, 1)),
            (t = Gt(t, e, 1)),
            (e = Ce()),
            t !== null && (Ci(t, 1, e), ze(t, e));
          break;
        }
      }
      t = t.return;
    }
}
function xy(e, t, n) {
  var r = e.pingCache;
  r !== null && r.delete(t),
    (t = Ce()),
    (e.pingedLanes |= e.suspendedLanes & n),
    pe === e &&
      (ge & n) === n &&
      (ce === 4 || (ce === 3 && (ge & 130023424) === ge && 500 > le() - na)
        ? hn(e, 0)
        : (ta |= n)),
    ze(e, t);
}
function bh(e, t) {
  t === 0 &&
    (e.mode & 1
      ? ((t = Ii), (Ii <<= 1), !(Ii & 130023424) && (Ii = 4194304))
      : (t = 1));
  var n = Ce();
  (e = _t(e, t)), e !== null && (Ci(e, t, n), ze(e, n));
}
function vy(e) {
  var t = e.memoizedState,
    n = 0;
  t !== null && (n = t.retryLane), bh(e, n);
}
function wy(e, t) {
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
      throw Error(M(314));
  }
  r !== null && r.delete(t), bh(e, n);
}
var ep;
ep = function (e, t, n) {
  if (e !== null)
    if (e.memoizedProps !== t.pendingProps || Ae.current) Ne = !0;
    else {
      if (!(e.lanes & n) && !(t.flags & 128)) return (Ne = !1), sy(e, t, n);
      Ne = !!(e.flags & 131072);
    }
  else (Ne = !1), ee && t.flags & 1048576 && ih(t, jo, t.index);
  switch (((t.lanes = 0), t.tag)) {
    case 2:
      var r = t.type;
      ao(e, t), (e = t.pendingProps);
      var i = lr(t, Ee.current);
      nr(t, n), (i = Ku(null, t, r, e, i, n));
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
            De(r) ? ((o = !0), Co(t)) : (o = !1),
            (t.memoizedState =
              i.state !== null && i.state !== void 0 ? i.state : null),
            Bu(t),
            (i.updater = ll),
            (t.stateNode = i),
            (i._reactInternals = t),
            Hs(t, r, e, n),
            (t = Bs(null, t, r, !0, o, n)))
          : ((t.tag = 0), ee && o && Ru(t), Pe(null, t, i, n), (t = t.child)),
        t
      );
    case 16:
      r = t.elementType;
      e: {
        switch (
          (ao(e, t),
          (e = t.pendingProps),
          (i = r._init),
          (r = i(r._payload)),
          (t.type = r),
          (i = t.tag = Sy(r)),
          (e = rt(r, e)),
          i)
        ) {
          case 0:
            t = Vs(null, t, r, e, n);
            break e;
          case 1:
            t = Lc(null, t, r, e, n);
            break e;
          case 11:
            t = Tc(null, t, r, e, n);
            break e;
          case 14:
            t = _c(null, t, r, rt(r.type, e), n);
            break e;
        }
        throw Error(M(306, r, ""));
      }
      return t;
    case 0:
      return (
        (r = t.type),
        (i = t.pendingProps),
        (i = t.elementType === r ? i : rt(r, i)),
        Vs(e, t, r, i, n)
      );
    case 1:
      return (
        (r = t.type),
        (i = t.pendingProps),
        (i = t.elementType === r ? i : rt(r, i)),
        Lc(e, t, r, i, n)
      );
    case 3:
      e: {
        if ((Rh(t), e === null)) throw Error(M(387));
        (r = t.pendingProps),
          (o = t.memoizedState),
          (i = o.element),
          ch(e, t),
          Lo(t, r, null, n);
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
            (i = cr(Error(M(423)), t)), (t = Nc(e, t, r, n, i));
            break e;
          } else if (r !== i) {
            (i = cr(Error(M(424)), t)), (t = Nc(e, t, r, n, i));
            break e;
          } else
            for (
              Ie = Yt(t.stateNode.containerInfo.firstChild),
                Ue = t,
                ee = !0,
                ot = null,
                n = uh(t, null, r, n),
                t.child = n;
              n;

            )
              (n.flags = (n.flags & -3) | 4096), (n = n.sibling);
        else {
          if ((sr(), r === i)) {
            t = Lt(e, t, n);
            break e;
          }
          Pe(e, t, r, n);
        }
        t = t.child;
      }
      return t;
    case 5:
      return (
        fh(t),
        e === null && Fs(t),
        (r = t.type),
        (i = t.pendingProps),
        (o = e !== null ? e.memoizedProps : null),
        (l = i.children),
        As(r, i) ? (l = null) : o !== null && As(r, o) && (t.flags |= 32),
        Oh(e, t),
        Pe(e, t, l, n),
        t.child
      );
    case 6:
      return e === null && Fs(t), null;
    case 13:
      return Fh(e, t, n);
    case 4:
      return (
        Yu(t, t.stateNode.containerInfo),
        (r = t.pendingProps),
        e === null ? (t.child = ur(t, null, r, n)) : Pe(e, t, r, n),
        t.child
      );
    case 11:
      return (
        (r = t.type),
        (i = t.pendingProps),
        (i = t.elementType === r ? i : rt(r, i)),
        Tc(e, t, r, i, n)
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
          q(To, r._currentValue),
          (r._currentValue = l),
          o !== null)
        )
          if (ct(o.value, l)) {
            if (o.children === i.children && !Ae.current) {
              t = Lt(e, t, n);
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
                        var c = a.pending;
                        c === null
                          ? (u.next = u)
                          : ((u.next = c.next), (c.next = u)),
                          (a.pending = u);
                      }
                    }
                    (o.lanes |= n),
                      (u = o.alternate),
                      u !== null && (u.lanes |= n),
                      Is(o.return, n, t),
                      (s.lanes |= n);
                    break;
                  }
                  u = u.next;
                }
              } else if (o.tag === 10) l = o.type === t.type ? null : o.child;
              else if (o.tag === 18) {
                if (((l = o.return), l === null)) throw Error(M(341));
                (l.lanes |= n),
                  (s = l.alternate),
                  s !== null && (s.lanes |= n),
                  Is(l, n, t),
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
        nr(t, n),
        (i = be(i)),
        (r = r(i)),
        (t.flags |= 1),
        Pe(e, t, r, n),
        t.child
      );
    case 14:
      return (
        (r = t.type),
        (i = rt(r, t.pendingProps)),
        (i = rt(r.type, i)),
        _c(e, t, r, i, n)
      );
    case 15:
      return Dh(e, t, t.type, t.pendingProps, n);
    case 17:
      return (
        (r = t.type),
        (i = t.pendingProps),
        (i = t.elementType === r ? i : rt(r, i)),
        ao(e, t),
        (t.tag = 1),
        De(r) ? ((e = !0), Co(t)) : (e = !1),
        nr(t, n),
        Nh(t, r, i),
        Hs(t, r, i, n),
        Bs(null, t, r, !0, e, n)
      );
    case 19:
      return Ih(e, t, n);
    case 22:
      return zh(e, t, n);
  }
  throw Error(M(156, t.tag));
};
function tp(e, t) {
  return Td(e, t);
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
function la(e) {
  return (e = e.prototype), !(!e || !e.isReactComponent);
}
function Sy(e) {
  if (typeof e == "function") return la(e) ? 1 : 0;
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
function ho(e, t, n, r, i, o) {
  var l = 2;
  if (((r = e), typeof e == "function")) la(e) && (l = 1);
  else if (typeof e == "string") l = 5;
  else
    e: switch (e) {
      case Fn:
        return pn(n.children, i, o, t);
      case Pu:
        (l = 8), (i |= 8);
        break;
      case fs:
        return (
          (e = Ze(12, n, t, i | 2)), (e.elementType = fs), (e.lanes = o), e
        );
      case ds:
        return (e = Ze(13, n, t, i)), (e.elementType = ds), (e.lanes = o), e;
      case hs:
        return (e = Ze(19, n, t, i)), (e.elementType = hs), (e.lanes = o), e;
      case fd:
        return al(n, i, o, t);
      default:
        if (typeof e == "object" && e !== null)
          switch (e.$$typeof) {
            case ad:
              l = 10;
              break e;
            case cd:
              l = 9;
              break e;
            case Cu:
              l = 11;
              break e;
            case Mu:
              l = 14;
              break e;
            case zt:
              (l = 16), (r = null);
              break e;
          }
        throw Error(M(130, e == null ? e : typeof e, ""));
    }
  return (
    (t = Ze(l, n, t, i)), (t.elementType = e), (t.type = r), (t.lanes = o), t
  );
}
function pn(e, t, n, r) {
  return (e = Ze(7, e, r, t)), (e.lanes = n), e;
}
function al(e, t, n, r) {
  return (
    (e = Ze(22, e, r, t)),
    (e.elementType = fd),
    (e.lanes = n),
    (e.stateNode = { isHidden: !1 }),
    e
  );
}
function Yl(e, t, n) {
  return (e = Ze(6, e, null, t)), (e.lanes = n), e;
}
function Gl(e, t, n) {
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
    (this.eventTimes = Ml(0)),
    (this.expirationTimes = Ml(-1)),
    (this.entangledLanes =
      this.finishedLanes =
      this.mutableReadLanes =
      this.expiredLanes =
      this.pingedLanes =
      this.suspendedLanes =
      this.pendingLanes =
        0),
    (this.entanglements = Ml(0)),
    (this.identifierPrefix = r),
    (this.onRecoverableError = i),
    (this.mutableSourceEagerHydrationData = null);
}
function sa(e, t, n, r, i, o, l, s, u) {
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
    Bu(o),
    e
  );
}
function Py(e, t, n) {
  var r = 3 < arguments.length && arguments[3] !== void 0 ? arguments[3] : null;
  return {
    $$typeof: Rn,
    key: r == null ? null : "" + r,
    children: e,
    containerInfo: t,
    implementation: n,
  };
}
function np(e) {
  if (!e) return qt;
  e = e._reactInternals;
  e: {
    if (Tn(e) !== e || e.tag !== 1) throw Error(M(170));
    var t = e;
    do {
      switch (t.tag) {
        case 3:
          t = t.stateNode.context;
          break e;
        case 1:
          if (De(t.type)) {
            t = t.stateNode.__reactInternalMemoizedMergedChildContext;
            break e;
          }
      }
      t = t.return;
    } while (t !== null);
    throw Error(M(171));
  }
  if (e.tag === 1) {
    var n = e.type;
    if (De(n)) return nh(e, n, t);
  }
  return t;
}
function rp(e, t, n, r, i, o, l, s, u) {
  return (
    (e = sa(n, r, !0, e, i, o, l, s, u)),
    (e.context = np(null)),
    (n = e.current),
    (r = Ce()),
    (i = Xt(n)),
    (o = Mt(r, i)),
    (o.callback = t ?? null),
    Gt(n, o, i),
    (e.current.lanes = i),
    Ci(e, i, r),
    ze(e, r),
    e
  );
}
function cl(e, t, n, r) {
  var i = t.current,
    o = Ce(),
    l = Xt(i);
  return (
    (n = np(n)),
    t.context === null ? (t.context = n) : (t.pendingContext = n),
    (t = Mt(o, l)),
    (t.payload = { element: e }),
    (r = r === void 0 ? null : r),
    r !== null && (t.callback = r),
    (e = Gt(i, t, l)),
    e !== null && (at(e, i, l, o), lo(e, i, l)),
    l
  );
}
function Fo(e) {
  if (((e = e.current), !e.child)) return null;
  switch (e.child.tag) {
    case 5:
      return e.child.stateNode;
    default:
      return e.child.stateNode;
  }
}
function Hc(e, t) {
  if (((e = e.memoizedState), e !== null && e.dehydrated !== null)) {
    var n = e.retryLane;
    e.retryLane = n !== 0 && n < t ? n : t;
  }
}
function ua(e, t) {
  Hc(e, t), (e = e.alternate) && Hc(e, t);
}
function Cy() {
  return null;
}
var ip =
  typeof reportError == "function"
    ? reportError
    : function (e) {
        console.error(e);
      };
function aa(e) {
  this._internalRoot = e;
}
fl.prototype.render = aa.prototype.render = function (e) {
  var t = this._internalRoot;
  if (t === null) throw Error(M(409));
  cl(e, t, null, null);
};
fl.prototype.unmount = aa.prototype.unmount = function () {
  var e = this._internalRoot;
  if (e !== null) {
    this._internalRoot = null;
    var t = e.containerInfo;
    Sn(function () {
      cl(null, e, null, null);
    }),
      (t[Tt] = null);
  }
};
function fl(e) {
  this._internalRoot = e;
}
fl.prototype.unstable_scheduleHydration = function (e) {
  if (e) {
    var t = zd();
    e = { blockedOn: null, target: e, priority: t };
    for (var n = 0; n < Ft.length && t !== 0 && t < Ft[n].priority; n++);
    Ft.splice(n, 0, e), n === 0 && Rd(e);
  }
};
function ca(e) {
  return !(!e || (e.nodeType !== 1 && e.nodeType !== 9 && e.nodeType !== 11));
}
function dl(e) {
  return !(
    !e ||
    (e.nodeType !== 1 &&
      e.nodeType !== 9 &&
      e.nodeType !== 11 &&
      (e.nodeType !== 8 || e.nodeValue !== " react-mount-point-unstable "))
  );
}
function Wc() {}
function My(e, t, n, r, i) {
  if (i) {
    if (typeof r == "function") {
      var o = r;
      r = function () {
        var a = Fo(l);
        o.call(a);
      };
    }
    var l = rp(t, r, e, 0, null, !1, !1, "", Wc);
    return (
      (e._reactRootContainer = l),
      (e[Tt] = l.current),
      ui(e.nodeType === 8 ? e.parentNode : e),
      Sn(),
      l
    );
  }
  for (; (i = e.lastChild); ) e.removeChild(i);
  if (typeof r == "function") {
    var s = r;
    r = function () {
      var a = Fo(u);
      s.call(a);
    };
  }
  var u = sa(e, 0, !1, null, null, !1, !1, "", Wc);
  return (
    (e._reactRootContainer = u),
    (e[Tt] = u.current),
    ui(e.nodeType === 8 ? e.parentNode : e),
    Sn(function () {
      cl(t, u, n, r);
    }),
    u
  );
}
function hl(e, t, n, r, i) {
  var o = n._reactRootContainer;
  if (o) {
    var l = o;
    if (typeof i == "function") {
      var s = i;
      i = function () {
        var u = Fo(l);
        s.call(u);
      };
    }
    cl(t, l, e, i);
  } else l = My(n, t, e, i, r);
  return Fo(l);
}
Ad = function (e) {
  switch (e.tag) {
    case 3:
      var t = e.stateNode;
      if (t.current.memoizedState.isDehydrated) {
        var n = Fr(t.pendingLanes);
        n !== 0 &&
          (_u(t, n | 1), ze(t, le()), !(W & 6) && ((fr = le() + 500), en()));
      }
      break;
    case 13:
      Sn(function () {
        var r = _t(e, 1);
        if (r !== null) {
          var i = Ce();
          at(r, e, 1, i);
        }
      }),
        ua(e, 1);
  }
};
Lu = function (e) {
  if (e.tag === 13) {
    var t = _t(e, 134217728);
    if (t !== null) {
      var n = Ce();
      at(t, e, 134217728, n);
    }
    ua(e, 134217728);
  }
};
Dd = function (e) {
  if (e.tag === 13) {
    var t = Xt(e),
      n = _t(e, t);
    if (n !== null) {
      var r = Ce();
      at(n, e, t, r);
    }
    ua(e, t);
  }
};
zd = function () {
  return G;
};
Od = function (e, t) {
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
      if ((ys(e, n), (t = n.name), n.type === "radio" && t != null)) {
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
            var i = rl(r);
            if (!i) throw Error(M(90));
            hd(r), ys(r, i);
          }
        }
      }
      break;
    case "textarea":
      md(e, n);
      break;
    case "select":
      (t = n.value), t != null && Jn(e, !!n.multiple, t, !1);
  }
};
Sd = ra;
Ed = Sn;
var jy = { usingClientEntryPoint: !1, Events: [ji, Wn, rl, wd, kd, ra] },
  Lr = {
    findFiberByHostInstance: an,
    bundleType: 0,
    version: "18.3.1",
    rendererPackageName: "react-dom",
  },
  Ty = {
    bundleType: Lr.bundleType,
    version: Lr.version,
    rendererPackageName: Lr.rendererPackageName,
    rendererConfig: Lr.rendererConfig,
    overrideHookState: null,
    overrideHookStateDeletePath: null,
    overrideHookStateRenamePath: null,
    overrideProps: null,
    overridePropsDeletePath: null,
    overridePropsRenamePath: null,
    setErrorHandler: null,
    setSuspenseHandler: null,
    scheduleUpdate: null,
    currentDispatcherRef: At.ReactCurrentDispatcher,
    findHostInstanceByFiber: function (e) {
      return (e = Md(e)), e === null ? null : e.stateNode;
    },
    findFiberByHostInstance: Lr.findFiberByHostInstance || Cy,
    findHostInstancesForRefresh: null,
    scheduleRefresh: null,
    scheduleRoot: null,
    setRefreshHandler: null,
    getCurrentFiber: null,
    reconcilerVersion: "18.3.1-next-f1338f8080-20240426",
  };
if (typeof __REACT_DEVTOOLS_GLOBAL_HOOK__ < "u") {
  var Zi = __REACT_DEVTOOLS_GLOBAL_HOOK__;
  if (!Zi.isDisabled && Zi.supportsFiber)
    try {
      (bo = Zi.inject(Ty)), (mt = Zi);
    } catch {}
}
Be.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED = jy;
Be.createPortal = function (e, t) {
  var n = 2 < arguments.length && arguments[2] !== void 0 ? arguments[2] : null;
  if (!ca(t)) throw Error(M(200));
  return Py(e, t, null, n);
};
Be.createRoot = function (e, t) {
  if (!ca(e)) throw Error(M(299));
  var n = !1,
    r = "",
    i = ip;
  return (
    t != null &&
      (t.unstable_strictMode === !0 && (n = !0),
      t.identifierPrefix !== void 0 && (r = t.identifierPrefix),
      t.onRecoverableError !== void 0 && (i = t.onRecoverableError)),
    (t = sa(e, 1, !1, null, null, n, !1, r, i)),
    (e[Tt] = t.current),
    ui(e.nodeType === 8 ? e.parentNode : e),
    new aa(t)
  );
};
Be.findDOMNode = function (e) {
  if (e == null) return null;
  if (e.nodeType === 1) return e;
  var t = e._reactInternals;
  if (t === void 0)
    throw typeof e.render == "function"
      ? Error(M(188))
      : ((e = Object.keys(e).join(",")), Error(M(268, e)));
  return (e = Md(t)), (e = e === null ? null : e.stateNode), e;
};
Be.flushSync = function (e) {
  return Sn(e);
};
Be.hydrate = function (e, t, n) {
  if (!dl(t)) throw Error(M(200));
  return hl(null, e, t, !0, n);
};
Be.hydrateRoot = function (e, t, n) {
  if (!ca(e)) throw Error(M(405));
  var r = (n != null && n.hydratedSources) || null,
    i = !1,
    o = "",
    l = ip;
  if (
    (n != null &&
      (n.unstable_strictMode === !0 && (i = !0),
      n.identifierPrefix !== void 0 && (o = n.identifierPrefix),
      n.onRecoverableError !== void 0 && (l = n.onRecoverableError)),
    (t = rp(t, null, e, 1, n ?? null, i, !1, o, l)),
    (e[Tt] = t.current),
    ui(e),
    r)
  )
    for (e = 0; e < r.length; e++)
      (n = r[e]),
        (i = n._getVersion),
        (i = i(n._source)),
        t.mutableSourceEagerHydrationData == null
          ? (t.mutableSourceEagerHydrationData = [n, i])
          : t.mutableSourceEagerHydrationData.push(n, i);
  return new fl(t);
};
Be.render = function (e, t, n) {
  if (!dl(t)) throw Error(M(200));
  return hl(null, e, t, !1, n);
};
Be.unmountComponentAtNode = function (e) {
  if (!dl(e)) throw Error(M(40));
  return e._reactRootContainer
    ? (Sn(function () {
        hl(null, null, e, !1, function () {
          (e._reactRootContainer = null), (e[Tt] = null);
        });
      }),
      !0)
    : !1;
};
Be.unstable_batchedUpdates = ra;
Be.unstable_renderSubtreeIntoContainer = function (e, t, n, r) {
  if (!dl(n)) throw Error(M(200));
  if (e == null || e._reactInternals === void 0) throw Error(M(38));
  return hl(e, t, n, !1, r);
};
Be.version = "18.3.1-next-f1338f8080-20240426";
function op() {
  if (
    !(
      typeof __REACT_DEVTOOLS_GLOBAL_HOOK__ > "u" ||
      typeof __REACT_DEVTOOLS_GLOBAL_HOOK__.checkDCE != "function"
    )
  )
    try {
      __REACT_DEVTOOLS_GLOBAL_HOOK__.checkDCE(op);
    } catch (e) {
      console.error(e);
    }
}
op(), (od.exports = Be);
var _y = od.exports,
  Vc = _y;
(as.createRoot = Vc.createRoot), (as.hydrateRoot = Vc.hydrateRoot);
const Bc = 0;
function Ly(e) {
  return h.jsx("div", {
    children: h.jsxs("table", {
      children: [
        h.jsxs("tr", {
          children: [
            h.jsx("th", { align: "left", children: " Wheel " }),
            h.jsx("th", { children: " Command [-]" }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "Left" }),
            h.jsx("td", {
              align: "center",
              children: e.robot.leftMotor.command.toFixed(Bc),
            }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "Right" }),
            h.jsx("td", {
              align: "center",
              children: e.robot.rightMotor.command.toFixed(Bc),
            }),
          ],
        }),
      ],
    }),
  });
}
function Ny(e) {
  const t = e.robot.distances,
    n = e.distancePlot.labels;
  return h.jsx("div", {
    children: h.jsxs("table", {
      children: [
        h.jsxs("tr", {
          children: [
            h.jsx("th", { align: "left", children: " Sensor " }),
            h.jsx("th", { children: " Distance [mm]" }),
          ],
        }),
        t.map((r, i) =>
          h.jsxs(
            "tr",
            {
              children: [
                h.jsx("td", { children: n[i] }),
                h.jsx("td", { align: "center", children: r }),
              ],
            },
            i
          )
        ),
      ],
    }),
  });
}
const An = 2;
function $y(e) {
  return h.jsx("div", {
    children: h.jsxs("table", {
      children: [
        h.jsxs("tr", {
          children: [
            h.jsx("th", { children: "Dimension" }),
            h.jsx("th", { children: "Linear [m/s^2] " }),
            h.jsx("th", { children: "Angular [rad/s]" }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "x" }),
            h.jsx("td", { children: e.acceleration.x.toFixed(An) }),
            h.jsx("td", { children: e.rotation.x.toFixed(An) }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "y" }),
            h.jsx("td", { children: e.acceleration.y.toFixed(An) }),
            h.jsx("td", { children: e.rotation.y.toFixed(An) }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "z" }),
            h.jsx("td", { children: e.acceleration.z.toFixed(An) }),
            h.jsx("td", { children: e.rotation.z.toFixed(An) }),
          ],
        }),
      ],
    }),
  });
}
function po(e, t) {
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
function Ay(e, t) {
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
function fa(e) {
  let t, n, r;
  e.length !== 2
    ? ((t = po), (n = (s, u) => po(e(s), u)), (r = (s, u) => e(s) - u))
    : ((t = e === po || e === Ay ? e : Dy), (n = e), (r = e));
  function i(s, u, a = 0, c = s.length) {
    if (a < c) {
      if (t(u, u) !== 0) return c;
      do {
        const f = (a + c) >>> 1;
        n(s[f], u) < 0 ? (a = f + 1) : (c = f);
      } while (a < c);
    }
    return a;
  }
  function o(s, u, a = 0, c = s.length) {
    if (a < c) {
      if (t(u, u) !== 0) return c;
      do {
        const f = (a + c) >>> 1;
        n(s[f], u) <= 0 ? (a = f + 1) : (c = f);
      } while (a < c);
    }
    return a;
  }
  function l(s, u, a = 0, c = s.length) {
    const f = i(s, u, a, c - 1);
    return f > a && r(s[f - 1], u) > -r(s[f], u) ? f - 1 : f;
  }
  return { left: i, center: l, right: o };
}
function Dy() {
  return 0;
}
function zy(e) {
  return e === null ? NaN : +e;
}
const Oy = fa(po),
  Ry = Oy.right;
fa(zy).center;
function br(e, t) {
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
class Yc extends Map {
  constructor(t, n = Uy) {
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
    return super.set(Fy(this, t), n);
  }
  delete(t) {
    return super.delete(Iy(this, t));
  }
}
function Gc({ _intern: e, _key: t }, n) {
  const r = t(n);
  return e.has(r) ? e.get(r) : n;
}
function Fy({ _intern: e, _key: t }, n) {
  const r = t(n);
  return e.has(r) ? e.get(r) : (e.set(r, n), n);
}
function Iy({ _intern: e, _key: t }, n) {
  const r = t(n);
  return e.has(r) && ((n = e.get(r)), e.delete(r)), n;
}
function Uy(e) {
  return e !== null && typeof e == "object" ? e.valueOf() : e;
}
const Hy = Math.sqrt(50),
  Wy = Math.sqrt(10),
  Vy = Math.sqrt(2);
function Io(e, t, n) {
  const r = (t - e) / Math.max(0, n),
    i = Math.floor(Math.log10(r)),
    o = r / Math.pow(10, i),
    l = o >= Hy ? 10 : o >= Wy ? 5 : o >= Vy ? 2 : 1;
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
    u < s && 0.5 <= n && n < 2 ? Io(e, t, n * 2) : [s, u, a]
  );
}
function nu(e, t, n) {
  if (((t = +t), (e = +e), (n = +n), !(n > 0))) return [];
  if (e === t) return [e];
  const r = t < e,
    [i, o, l] = r ? Io(t, e, n) : Io(e, t, n);
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
function ru(e, t, n) {
  return (t = +t), (e = +e), (n = +n), Io(e, t, n)[2];
}
function iu(e, t, n) {
  (t = +t), (e = +e), (n = +n);
  const r = t < e,
    i = r ? ru(t, e, n) : ru(e, t, n);
  return (r ? -1 : 1) * (i < 0 ? 1 / -i : i);
}
function By(e, t) {
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
function Yy(e, t) {
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
function pl(e, t) {
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
const Qc = Symbol("implicit");
function da() {
  var e = new Yc(),
    t = [],
    n = [],
    r = Qc;
  function i(o) {
    let l = e.get(o);
    if (l === void 0) {
      if (r !== Qc) return r;
      e.set(o, (l = t.push(o) - 1));
    }
    return n[l % n.length];
  }
  return (
    (i.domain = function (o) {
      if (!arguments.length) return t.slice();
      (t = []), (e = new Yc());
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
      return da(t, n).unknown(r);
    }),
    pl.apply(i, arguments),
    i
  );
}
function ha(e, t, n) {
  (e.prototype = t.prototype = n), (n.constructor = e);
}
function lp(e, t) {
  var n = Object.create(e.prototype);
  for (var r in t) n[r] = t[r];
  return n;
}
function _i() {}
var gi = 0.7,
  Uo = 1 / gi,
  ir = "\\s*([+-]?\\d+)\\s*",
  xi = "\\s*([+-]?(?:\\d*\\.)?\\d+(?:[eE][+-]?\\d+)?)\\s*",
  gt = "\\s*([+-]?(?:\\d*\\.)?\\d+(?:[eE][+-]?\\d+)?)%\\s*",
  Gy = /^#([0-9a-f]{3,8})$/,
  Qy = new RegExp(`^rgb\\(${ir},${ir},${ir}\\)$`),
  Xy = new RegExp(`^rgb\\(${gt},${gt},${gt}\\)$`),
  Ky = new RegExp(`^rgba\\(${ir},${ir},${ir},${xi}\\)$`),
  Zy = new RegExp(`^rgba\\(${gt},${gt},${gt},${xi}\\)$`),
  qy = new RegExp(`^hsl\\(${xi},${gt},${gt}\\)$`),
  Jy = new RegExp(`^hsla\\(${xi},${gt},${gt},${xi}\\)$`),
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
ha(_i, vi, {
  copy(e) {
    return Object.assign(new this.constructor(), this, e);
  },
  displayable() {
    return this.rgb().displayable();
  },
  hex: Kc,
  formatHex: Kc,
  formatHex8: by,
  formatHsl: eg,
  formatRgb: Zc,
  toString: Zc,
});
function Kc() {
  return this.rgb().formatHex();
}
function by() {
  return this.rgb().formatHex8();
}
function eg() {
  return sp(this).formatHsl();
}
function Zc() {
  return this.rgb().formatRgb();
}
function vi(e) {
  var t, n;
  return (
    (e = (e + "").trim().toLowerCase()),
    (t = Gy.exec(e))
      ? ((n = t[1].length),
        (t = parseInt(t[1], 16)),
        n === 6
          ? qc(t)
          : n === 3
          ? new $e(
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
      : (t = Qy.exec(e))
      ? new $e(t[1], t[2], t[3], 1)
      : (t = Xy.exec(e))
      ? new $e((t[1] * 255) / 100, (t[2] * 255) / 100, (t[3] * 255) / 100, 1)
      : (t = Ky.exec(e))
      ? qi(t[1], t[2], t[3], t[4])
      : (t = Zy.exec(e))
      ? qi((t[1] * 255) / 100, (t[2] * 255) / 100, (t[3] * 255) / 100, t[4])
      : (t = qy.exec(e))
      ? ef(t[1], t[2] / 100, t[3] / 100, 1)
      : (t = Jy.exec(e))
      ? ef(t[1], t[2] / 100, t[3] / 100, t[4])
      : Xc.hasOwnProperty(e)
      ? qc(Xc[e])
      : e === "transparent"
      ? new $e(NaN, NaN, NaN, 0)
      : null
  );
}
function qc(e) {
  return new $e((e >> 16) & 255, (e >> 8) & 255, e & 255, 1);
}
function qi(e, t, n, r) {
  return r <= 0 && (e = t = n = NaN), new $e(e, t, n, r);
}
function tg(e) {
  return (
    e instanceof _i || (e = vi(e)),
    e ? ((e = e.rgb()), new $e(e.r, e.g, e.b, e.opacity)) : new $e()
  );
}
function ou(e, t, n, r) {
  return arguments.length === 1 ? tg(e) : new $e(e, t, n, r ?? 1);
}
function $e(e, t, n, r) {
  (this.r = +e), (this.g = +t), (this.b = +n), (this.opacity = +r);
}
ha(
  $e,
  ou,
  lp(_i, {
    brighter(e) {
      return (
        (e = e == null ? Uo : Math.pow(Uo, e)),
        new $e(this.r * e, this.g * e, this.b * e, this.opacity)
      );
    },
    darker(e) {
      return (
        (e = e == null ? gi : Math.pow(gi, e)),
        new $e(this.r * e, this.g * e, this.b * e, this.opacity)
      );
    },
    rgb() {
      return this;
    },
    clamp() {
      return new $e(mn(this.r), mn(this.g), mn(this.b), Ho(this.opacity));
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
    hex: Jc,
    formatHex: Jc,
    formatHex8: ng,
    formatRgb: bc,
    toString: bc,
  })
);
function Jc() {
  return `#${dn(this.r)}${dn(this.g)}${dn(this.b)}`;
}
function ng() {
  return `#${dn(this.r)}${dn(this.g)}${dn(this.b)}${dn(
    (isNaN(this.opacity) ? 1 : this.opacity) * 255
  )}`;
}
function bc() {
  const e = Ho(this.opacity);
  return `${e === 1 ? "rgb(" : "rgba("}${mn(this.r)}, ${mn(this.g)}, ${mn(
    this.b
  )}${e === 1 ? ")" : `, ${e})`}`;
}
function Ho(e) {
  return isNaN(e) ? 1 : Math.max(0, Math.min(1, e));
}
function mn(e) {
  return Math.max(0, Math.min(255, Math.round(e) || 0));
}
function dn(e) {
  return (e = mn(e)), (e < 16 ? "0" : "") + e.toString(16);
}
function ef(e, t, n, r) {
  return (
    r <= 0
      ? (e = t = n = NaN)
      : n <= 0 || n >= 1
      ? (e = t = NaN)
      : t <= 0 && (e = NaN),
    new lt(e, t, n, r)
  );
}
function sp(e) {
  if (e instanceof lt) return new lt(e.h, e.s, e.l, e.opacity);
  if ((e instanceof _i || (e = vi(e)), !e)) return new lt();
  if (e instanceof lt) return e;
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
    new lt(l, s, u, e.opacity)
  );
}
function rg(e, t, n, r) {
  return arguments.length === 1 ? sp(e) : new lt(e, t, n, r ?? 1);
}
function lt(e, t, n, r) {
  (this.h = +e), (this.s = +t), (this.l = +n), (this.opacity = +r);
}
ha(
  lt,
  rg,
  lp(_i, {
    brighter(e) {
      return (
        (e = e == null ? Uo : Math.pow(Uo, e)),
        new lt(this.h, this.s, this.l * e, this.opacity)
      );
    },
    darker(e) {
      return (
        (e = e == null ? gi : Math.pow(gi, e)),
        new lt(this.h, this.s, this.l * e, this.opacity)
      );
    },
    rgb() {
      var e = (this.h % 360) + (this.h < 0) * 360,
        t = isNaN(e) || isNaN(this.s) ? 0 : this.s,
        n = this.l,
        r = n + (n < 0.5 ? n : 1 - n) * t,
        i = 2 * n - r;
      return new $e(
        Ql(e >= 240 ? e - 240 : e + 120, i, r),
        Ql(e, i, r),
        Ql(e < 120 ? e + 240 : e - 120, i, r),
        this.opacity
      );
    },
    clamp() {
      return new lt(tf(this.h), Ji(this.s), Ji(this.l), Ho(this.opacity));
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
      const e = Ho(this.opacity);
      return `${e === 1 ? "hsl(" : "hsla("}${tf(this.h)}, ${
        Ji(this.s) * 100
      }%, ${Ji(this.l) * 100}%${e === 1 ? ")" : `, ${e})`}`;
    },
  })
);
function tf(e) {
  return (e = (e || 0) % 360), e < 0 ? e + 360 : e;
}
function Ji(e) {
  return Math.max(0, Math.min(1, e || 0));
}
function Ql(e, t, n) {
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
const pa = (e) => () => e;
function ig(e, t) {
  return function (n) {
    return e + n * t;
  };
}
function og(e, t, n) {
  return (
    (e = Math.pow(e, n)),
    (t = Math.pow(t, n) - e),
    (n = 1 / n),
    function (r) {
      return Math.pow(e + r * t, n);
    }
  );
}
function lg(e) {
  return (e = +e) == 1
    ? up
    : function (t, n) {
        return n - t ? og(t, n, e) : pa(isNaN(t) ? n : t);
      };
}
function up(e, t) {
  var n = t - e;
  return n ? ig(e, n) : pa(isNaN(e) ? t : e);
}
const nf = (function e(t) {
  var n = lg(t);
  function r(i, o) {
    var l = n((i = ou(i)).r, (o = ou(o)).r),
      s = n(i.g, o.g),
      u = n(i.b, o.b),
      a = up(i.opacity, o.opacity);
    return function (c) {
      return (
        (i.r = l(c)), (i.g = s(c)), (i.b = u(c)), (i.opacity = a(c)), i + ""
      );
    };
  }
  return (r.gamma = e), r;
})(1);
function sg(e, t) {
  t || (t = []);
  var n = e ? Math.min(t.length, e.length) : 0,
    r = t.slice(),
    i;
  return function (o) {
    for (i = 0; i < n; ++i) r[i] = e[i] * (1 - o) + t[i] * o;
    return r;
  };
}
function ug(e) {
  return ArrayBuffer.isView(e) && !(e instanceof DataView);
}
function ag(e, t) {
  var n = t ? t.length : 0,
    r = e ? Math.min(n, e.length) : 0,
    i = new Array(r),
    o = new Array(n),
    l;
  for (l = 0; l < r; ++l) i[l] = ma(e[l], t[l]);
  for (; l < n; ++l) o[l] = t[l];
  return function (s) {
    for (l = 0; l < r; ++l) o[l] = i[l](s);
    return o;
  };
}
function cg(e, t) {
  var n = new Date();
  return (
    (e = +e),
    (t = +t),
    function (r) {
      return n.setTime(e * (1 - r) + t * r), n;
    }
  );
}
function Wo(e, t) {
  return (
    (e = +e),
    (t = +t),
    function (n) {
      return e * (1 - n) + t * n;
    }
  );
}
function fg(e, t) {
  var n = {},
    r = {},
    i;
  (e === null || typeof e != "object") && (e = {}),
    (t === null || typeof t != "object") && (t = {});
  for (i in t) i in e ? (n[i] = ma(e[i], t[i])) : (r[i] = t[i]);
  return function (o) {
    for (i in n) r[i] = n[i](o);
    return r;
  };
}
var lu = /[-+]?(?:\d+\.?\d*|\.?\d+)(?:[eE][-+]?\d+)?/g,
  Xl = new RegExp(lu.source, "g");
function dg(e) {
  return function () {
    return e;
  };
}
function hg(e) {
  return function (t) {
    return e(t) + "";
  };
}
function pg(e, t) {
  var n = (lu.lastIndex = Xl.lastIndex = 0),
    r,
    i,
    o,
    l = -1,
    s = [],
    u = [];
  for (e = e + "", t = t + ""; (r = lu.exec(e)) && (i = Xl.exec(t)); )
    (o = i.index) > n &&
      ((o = t.slice(n, o)), s[l] ? (s[l] += o) : (s[++l] = o)),
      (r = r[0]) === (i = i[0])
        ? s[l]
          ? (s[l] += i)
          : (s[++l] = i)
        : ((s[++l] = null), u.push({ i: l, x: Wo(r, i) })),
      (n = Xl.lastIndex);
  return (
    n < t.length && ((o = t.slice(n)), s[l] ? (s[l] += o) : (s[++l] = o)),
    s.length < 2
      ? u[0]
        ? hg(u[0].x)
        : dg(t)
      : ((t = u.length),
        function (a) {
          for (var c = 0, f; c < t; ++c) s[(f = u[c]).i] = f.x(a);
          return s.join("");
        })
  );
}
function ma(e, t) {
  var n = typeof t,
    r;
  return t == null || n === "boolean"
    ? pa(t)
    : (n === "number"
        ? Wo
        : n === "string"
        ? (r = vi(t))
          ? ((t = r), nf)
          : pg
        : t instanceof vi
        ? nf
        : t instanceof Date
        ? cg
        : ug(t)
        ? sg
        : Array.isArray(t)
        ? ag
        : (typeof t.valueOf != "function" && typeof t.toString != "function") ||
          isNaN(t)
        ? fg
        : Wo)(e, t);
}
function mg(e, t) {
  return (
    (e = +e),
    (t = +t),
    function (n) {
      return Math.round(e * (1 - n) + t * n);
    }
  );
}
function yg(e) {
  return function () {
    return e;
  };
}
function gg(e) {
  return +e;
}
var rf = [0, 1];
function Kn(e) {
  return e;
}
function su(e, t) {
  return (t -= e = +e)
    ? function (n) {
        return (n - e) / t;
      }
    : yg(isNaN(t) ? NaN : 0.5);
}
function xg(e, t) {
  var n;
  return (
    e > t && ((n = e), (e = t), (t = n)),
    function (r) {
      return Math.max(e, Math.min(t, r));
    }
  );
}
function vg(e, t, n) {
  var r = e[0],
    i = e[1],
    o = t[0],
    l = t[1];
  return (
    i < r ? ((r = su(i, r)), (o = n(l, o))) : ((r = su(r, i)), (o = n(o, l))),
    function (s) {
      return o(r(s));
    }
  );
}
function wg(e, t, n) {
  var r = Math.min(e.length, t.length) - 1,
    i = new Array(r),
    o = new Array(r),
    l = -1;
  for (
    e[r] < e[0] && ((e = e.slice().reverse()), (t = t.slice().reverse()));
    ++l < r;

  )
    (i[l] = su(e[l], e[l + 1])), (o[l] = n(t[l], t[l + 1]));
  return function (s) {
    var u = Ry(e, s, 1, r) - 1;
    return o[u](i[u](s));
  };
}
function ya(e, t) {
  return t
    .domain(e.domain())
    .range(e.range())
    .interpolate(e.interpolate())
    .clamp(e.clamp())
    .unknown(e.unknown());
}
function ap() {
  var e = rf,
    t = rf,
    n = ma,
    r,
    i,
    o,
    l = Kn,
    s,
    u,
    a;
  function c() {
    var d = Math.min(e.length, t.length);
    return (
      l !== Kn && (l = xg(e[0], e[d - 1])),
      (s = d > 2 ? wg : vg),
      (u = a = null),
      f
    );
  }
  function f(d) {
    return d == null || isNaN((d = +d))
      ? o
      : (u || (u = s(e.map(r), t, n)))(r(l(d)));
  }
  return (
    (f.invert = function (d) {
      return l(i((a || (a = s(t, e.map(r), Wo)))(d)));
    }),
    (f.domain = function (d) {
      return arguments.length ? ((e = Array.from(d, gg)), c()) : e.slice();
    }),
    (f.range = function (d) {
      return arguments.length ? ((t = Array.from(d)), c()) : t.slice();
    }),
    (f.rangeRound = function (d) {
      return (t = Array.from(d)), (n = mg), c();
    }),
    (f.clamp = function (d) {
      return arguments.length ? ((l = d ? !0 : Kn), c()) : l !== Kn;
    }),
    (f.interpolate = function (d) {
      return arguments.length ? ((n = d), c()) : n;
    }),
    (f.unknown = function (d) {
      return arguments.length ? ((o = d), f) : o;
    }),
    function (d, x) {
      return (r = d), (i = x), c();
    }
  );
}
function cp() {
  return ap()(Kn, Kn);
}
function kg(e) {
  return Math.abs((e = Math.round(e))) >= 1e21
    ? e.toLocaleString("en").replace(/,/g, "")
    : e.toString(10);
}
function Vo(e, t) {
  if (
    (n = (e = t ? e.toExponential(t - 1) : e.toExponential()).indexOf("e")) < 0
  )
    return null;
  var n,
    r = e.slice(0, n);
  return [r.length > 1 ? r[0] + r.slice(2) : r, +e.slice(n + 1)];
}
function dr(e) {
  return (e = Vo(Math.abs(e))), e ? e[1] : NaN;
}
function Sg(e, t) {
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
function wi(e) {
  if (!(t = Pg.exec(e))) throw new Error("invalid format: " + e);
  var t;
  return new ga({
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
wi.prototype = ga.prototype;
function ga(e) {
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
ga.prototype.toString = function () {
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
var fp;
function Mg(e, t) {
  var n = Vo(e, t);
  if (!n) return e + "";
  var r = n[0],
    i = n[1],
    o = i - (fp = Math.max(-8, Math.min(8, Math.floor(i / 3))) * 3) + 1,
    l = r.length;
  return o === l
    ? r
    : o > l
    ? r + new Array(o - l + 1).join("0")
    : o > 0
    ? r.slice(0, o) + "." + r.slice(o)
    : "0." + new Array(1 - o).join("0") + Vo(e, Math.max(0, t + o - 1))[0];
}
function of(e, t) {
  var n = Vo(e, t);
  if (!n) return e + "";
  var r = n[0],
    i = n[1];
  return i < 0
    ? "0." + new Array(-i).join("0") + r
    : r.length > i + 1
    ? r.slice(0, i + 1) + "." + r.slice(i + 1)
    : r + new Array(i - r.length + 2).join("0");
}
const lf = {
  "%": (e, t) => (e * 100).toFixed(t),
  b: (e) => Math.round(e).toString(2),
  c: (e) => e + "",
  d: kg,
  e: (e, t) => e.toExponential(t),
  f: (e, t) => e.toFixed(t),
  g: (e, t) => e.toPrecision(t),
  o: (e) => Math.round(e).toString(8),
  p: (e, t) => of(e * 100, t),
  r: of,
  s: Mg,
  X: (e) => Math.round(e).toString(16).toUpperCase(),
  x: (e) => Math.round(e).toString(16),
};
function sf(e) {
  return e;
}
var uf = Array.prototype.map,
  af = [
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
function jg(e) {
  var t =
      e.grouping === void 0 || e.thousands === void 0
        ? sf
        : Sg(uf.call(e.grouping, Number), e.thousands + ""),
    n = e.currency === void 0 ? "" : e.currency[0] + "",
    r = e.currency === void 0 ? "" : e.currency[1] + "",
    i = e.decimal === void 0 ? "." : e.decimal + "",
    o = e.numerals === void 0 ? sf : Eg(uf.call(e.numerals, String)),
    l = e.percent === void 0 ? "%" : e.percent + "",
    s = e.minus === void 0 ? "−" : e.minus + "",
    u = e.nan === void 0 ? "NaN" : e.nan + "";
  function a(f) {
    f = wi(f);
    var d = f.fill,
      x = f.align,
      g = f.sign,
      v = f.symbol,
      E = f.zero,
      m = f.width,
      p = f.comma,
      y = f.precision,
      w = f.trim,
      k = f.type;
    k === "n"
      ? ((p = !0), (k = "g"))
      : lf[k] || (y === void 0 && (y = 12), (w = !0), (k = "g")),
      (E || (d === "0" && x === "=")) && ((E = !0), (d = "0"), (x = "="));
    var S =
        v === "$"
          ? n
          : v === "#" && /[boxX]/.test(k)
          ? "0" + k.toLowerCase()
          : "",
      P = v === "$" ? r : /[%p]/.test(k) ? l : "",
      C = lf[k],
      D = /[defgprs%]/.test(k);
    y =
      y === void 0
        ? 6
        : /[gprs]/.test(k)
        ? Math.max(1, Math.min(21, y))
        : Math.max(0, Math.min(20, y));
    function O(N) {
      var H = S,
        R = P,
        Y,
        K,
        Q;
      if (k === "c") (R = C(N) + R), (N = "");
      else {
        N = +N;
        var oe = N < 0 || 1 / N < 0;
        if (
          ((N = isNaN(N) ? u : C(Math.abs(N), y)),
          w && (N = Cg(N)),
          oe && +N == 0 && g !== "+" && (oe = !1),
          (H =
            (oe ? (g === "(" ? g : s) : g === "-" || g === "(" ? "" : g) + H),
          (R =
            (k === "s" ? af[8 + fp / 3] : "") +
            R +
            (oe && g === "(" ? ")" : "")),
          D)
        ) {
          for (Y = -1, K = N.length; ++Y < K; )
            if (((Q = N.charCodeAt(Y)), 48 > Q || Q > 57)) {
              (R = (Q === 46 ? i + N.slice(Y + 1) : N.slice(Y)) + R),
                (N = N.slice(0, Y));
              break;
            }
        }
      }
      p && !E && (N = t(N, 1 / 0));
      var T = H.length + N.length + R.length,
        L = T < m ? new Array(m - T + 1).join(d) : "";
      switch (
        (p && E && ((N = t(L + N, L.length ? m - R.length : 1 / 0)), (L = "")),
        x)
      ) {
        case "<":
          N = H + N + R + L;
          break;
        case "=":
          N = H + L + N + R;
          break;
        case "^":
          N = L.slice(0, (T = L.length >> 1)) + H + N + R + L.slice(T);
          break;
        default:
          N = L + H + N + R;
          break;
      }
      return o(N);
    }
    return (
      (O.toString = function () {
        return f + "";
      }),
      O
    );
  }
  function c(f, d) {
    var x = a(((f = wi(f)), (f.type = "f"), f)),
      g = Math.max(-8, Math.min(8, Math.floor(dr(d) / 3))) * 3,
      v = Math.pow(10, -g),
      E = af[8 + g / 3];
    return function (m) {
      return x(v * m) + E;
    };
  }
  return { format: a, formatPrefix: c };
}
var bi, xa, dp;
Tg({ thousands: ",", grouping: [3], currency: ["$", ""] });
function Tg(e) {
  return (bi = jg(e)), (xa = bi.format), (dp = bi.formatPrefix), bi;
}
function _g(e) {
  return Math.max(0, -dr(Math.abs(e)));
}
function Lg(e, t) {
  return Math.max(
    0,
    Math.max(-8, Math.min(8, Math.floor(dr(t) / 3))) * 3 - dr(Math.abs(e))
  );
}
function Ng(e, t) {
  return (
    (e = Math.abs(e)), (t = Math.abs(t) - e), Math.max(0, dr(t) - dr(e)) + 1
  );
}
function $g(e, t, n, r) {
  var i = iu(e, t, n),
    o;
  switch (((r = wi(r ?? ",f")), r.type)) {
    case "s": {
      var l = Math.max(Math.abs(e), Math.abs(t));
      return (
        r.precision == null && !isNaN((o = Lg(i, l))) && (r.precision = o),
        dp(r, l)
      );
    }
    case "":
    case "e":
    case "g":
    case "p":
    case "r": {
      r.precision == null &&
        !isNaN((o = Ng(i, Math.max(Math.abs(e), Math.abs(t))))) &&
        (r.precision = o - (r.type === "e"));
      break;
    }
    case "f":
    case "%": {
      r.precision == null &&
        !isNaN((o = _g(i))) &&
        (r.precision = o - (r.type === "%") * 2);
      break;
    }
  }
  return xa(r);
}
function Ag(e) {
  var t = e.domain;
  return (
    (e.ticks = function (n) {
      var r = t();
      return nu(r[0], r[r.length - 1], n ?? 10);
    }),
    (e.tickFormat = function (n, r) {
      var i = t();
      return $g(i[0], i[i.length - 1], n ?? 10, r);
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
        c = 10;
      for (
        s < l && ((a = l), (l = s), (s = a), (a = i), (i = o), (o = a));
        c-- > 0;

      ) {
        if (((a = ru(l, s, n)), a === u)) return (r[i] = l), (r[o] = s), t(r);
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
function hp() {
  var e = cp();
  return (
    (e.copy = function () {
      return ya(e, hp());
    }),
    pl.apply(e, arguments),
    Ag(e)
  );
}
function pp(e, t) {
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
function cf(e) {
  return Math.log(e);
}
function ff(e) {
  return Math.exp(e);
}
function Dg(e) {
  return -Math.log(-e);
}
function zg(e) {
  return -Math.exp(-e);
}
function Og(e) {
  return isFinite(e) ? +("1e" + e) : e < 0 ? 0 : e;
}
function Rg(e) {
  return e === 10 ? Og : e === Math.E ? Math.exp : (t) => Math.pow(e, t);
}
function Fg(e) {
  return e === Math.E
    ? Math.log
    : (e === 10 && Math.log10) ||
        (e === 2 && Math.log2) ||
        ((e = Math.log(e)), (t) => Math.log(t) / e);
}
function df(e) {
  return (t, n) => -e(-t, n);
}
function Ig(e) {
  const t = e(cf, ff),
    n = t.domain;
  let r = 10,
    i,
    o;
  function l() {
    return (
      (i = Fg(r)),
      (o = Rg(r)),
      n()[0] < 0 ? ((i = df(i)), (o = df(o)), e(Dg, zg)) : e(cf, ff),
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
        c = u[u.length - 1];
      const f = c < a;
      f && ([a, c] = [c, a]);
      let d = i(a),
        x = i(c),
        g,
        v;
      const E = s == null ? 10 : +s;
      let m = [];
      if (!(r % 1) && x - d < E) {
        if (((d = Math.floor(d)), (x = Math.ceil(x)), a > 0)) {
          for (; d <= x; ++d)
            for (g = 1; g < r; ++g)
              if (((v = d < 0 ? g / o(-d) : g * o(d)), !(v < a))) {
                if (v > c) break;
                m.push(v);
              }
        } else
          for (; d <= x; ++d)
            for (g = r - 1; g >= 1; --g)
              if (((v = d > 0 ? g / o(-d) : g * o(d)), !(v < a))) {
                if (v > c) break;
                m.push(v);
              }
        m.length * 2 < E && (m = nu(a, c, E));
      } else m = nu(d, x, Math.min(x - d, E)).map(o);
      return f ? m.reverse() : m;
    }),
    (t.tickFormat = (s, u) => {
      if (
        (s == null && (s = 10),
        u == null && (u = r === 10 ? "s" : ","),
        typeof u != "function" &&
          (!(r % 1) && (u = wi(u)).precision == null && (u.trim = !0),
          (u = xa(u))),
        s === 1 / 0)
      )
        return u;
      const a = Math.max(1, (r * s) / t.ticks().length);
      return (c) => {
        let f = c / o(Math.round(i(c)));
        return f * r < r - 0.5 && (f *= r), f <= a ? u(c) : "";
      };
    }),
    (t.nice = () =>
      n(
        pp(n(), {
          floor: (s) => o(Math.floor(i(s))),
          ceil: (s) => o(Math.ceil(i(s))),
        })
      )),
    t
  );
}
function mp() {
  const e = Ig(ap()).domain([1, 10]);
  return (e.copy = () => ya(e, mp()).base(e.base())), pl.apply(e, arguments), e;
}
const Kl = new Date(),
  Zl = new Date();
function fe(e, t, n, r) {
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
      fe(
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
        Kl.setTime(+o), Zl.setTime(+l), e(Kl), e(Zl), Math.floor(n(Kl, Zl))
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
const Bo = fe(
  () => {},
  (e, t) => {
    e.setTime(+e + t);
  },
  (e, t) => t - e
);
Bo.every = (e) => (
  (e = Math.floor(e)),
  !isFinite(e) || !(e > 0)
    ? null
    : e > 1
    ? fe(
        (t) => {
          t.setTime(Math.floor(t / e) * e);
        },
        (t, n) => {
          t.setTime(+t + n * e);
        },
        (t, n) => (n - t) / e
      )
    : Bo
);
Bo.range;
const Pt = 1e3,
  qe = Pt * 60,
  Ct = qe * 60,
  Nt = Ct * 24,
  va = Nt * 7,
  hf = Nt * 30,
  ql = Nt * 365,
  Zn = fe(
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
const wa = fe(
  (e) => {
    e.setTime(e - e.getMilliseconds() - e.getSeconds() * Pt);
  },
  (e, t) => {
    e.setTime(+e + t * qe);
  },
  (e, t) => (t - e) / qe,
  (e) => e.getMinutes()
);
wa.range;
const Ug = fe(
  (e) => {
    e.setUTCSeconds(0, 0);
  },
  (e, t) => {
    e.setTime(+e + t * qe);
  },
  (e, t) => (t - e) / qe,
  (e) => e.getUTCMinutes()
);
Ug.range;
const ka = fe(
  (e) => {
    e.setTime(
      e - e.getMilliseconds() - e.getSeconds() * Pt - e.getMinutes() * qe
    );
  },
  (e, t) => {
    e.setTime(+e + t * Ct);
  },
  (e, t) => (t - e) / Ct,
  (e) => e.getHours()
);
ka.range;
const Hg = fe(
  (e) => {
    e.setUTCMinutes(0, 0, 0);
  },
  (e, t) => {
    e.setTime(+e + t * Ct);
  },
  (e, t) => (t - e) / Ct,
  (e) => e.getUTCHours()
);
Hg.range;
const Li = fe(
  (e) => e.setHours(0, 0, 0, 0),
  (e, t) => e.setDate(e.getDate() + t),
  (e, t) => (t - e - (t.getTimezoneOffset() - e.getTimezoneOffset()) * qe) / Nt,
  (e) => e.getDate() - 1
);
Li.range;
const Sa = fe(
  (e) => {
    e.setUTCHours(0, 0, 0, 0);
  },
  (e, t) => {
    e.setUTCDate(e.getUTCDate() + t);
  },
  (e, t) => (t - e) / Nt,
  (e) => e.getUTCDate() - 1
);
Sa.range;
const Wg = fe(
  (e) => {
    e.setUTCHours(0, 0, 0, 0);
  },
  (e, t) => {
    e.setUTCDate(e.getUTCDate() + t);
  },
  (e, t) => (t - e) / Nt,
  (e) => Math.floor(e / Nt)
);
Wg.range;
function _n(e) {
  return fe(
    (t) => {
      t.setDate(t.getDate() - ((t.getDay() + 7 - e) % 7)),
        t.setHours(0, 0, 0, 0);
    },
    (t, n) => {
      t.setDate(t.getDate() + n * 7);
    },
    (t, n) =>
      (n - t - (n.getTimezoneOffset() - t.getTimezoneOffset()) * qe) / va
  );
}
const ml = _n(0),
  Yo = _n(1),
  Vg = _n(2),
  Bg = _n(3),
  hr = _n(4),
  Yg = _n(5),
  Gg = _n(6);
ml.range;
Yo.range;
Vg.range;
Bg.range;
hr.range;
Yg.range;
Gg.range;
function Ln(e) {
  return fe(
    (t) => {
      t.setUTCDate(t.getUTCDate() - ((t.getUTCDay() + 7 - e) % 7)),
        t.setUTCHours(0, 0, 0, 0);
    },
    (t, n) => {
      t.setUTCDate(t.getUTCDate() + n * 7);
    },
    (t, n) => (n - t) / va
  );
}
const yp = Ln(0),
  Go = Ln(1),
  Qg = Ln(2),
  Xg = Ln(3),
  pr = Ln(4),
  Kg = Ln(5),
  Zg = Ln(6);
yp.range;
Go.range;
Qg.range;
Xg.range;
pr.range;
Kg.range;
Zg.range;
const Ea = fe(
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
const qg = fe(
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
qg.range;
const $t = fe(
  (e) => {
    e.setMonth(0, 1), e.setHours(0, 0, 0, 0);
  },
  (e, t) => {
    e.setFullYear(e.getFullYear() + t);
  },
  (e, t) => t.getFullYear() - e.getFullYear(),
  (e) => e.getFullYear()
);
$t.every = (e) =>
  !isFinite((e = Math.floor(e))) || !(e > 0)
    ? null
    : fe(
        (t) => {
          t.setFullYear(Math.floor(t.getFullYear() / e) * e),
            t.setMonth(0, 1),
            t.setHours(0, 0, 0, 0);
        },
        (t, n) => {
          t.setFullYear(t.getFullYear() + n * e);
        }
      );
$t.range;
const En = fe(
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
    : fe(
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
function Jg(e, t, n, r, i, o) {
  const l = [
    [Zn, 1, Pt],
    [Zn, 5, 5 * Pt],
    [Zn, 15, 15 * Pt],
    [Zn, 30, 30 * Pt],
    [o, 1, qe],
    [o, 5, 5 * qe],
    [o, 15, 15 * qe],
    [o, 30, 30 * qe],
    [i, 1, Ct],
    [i, 3, 3 * Ct],
    [i, 6, 6 * Ct],
    [i, 12, 12 * Ct],
    [r, 1, Nt],
    [r, 2, 2 * Nt],
    [n, 1, va],
    [t, 1, hf],
    [t, 3, 3 * hf],
    [e, 1, ql],
  ];
  function s(a, c, f) {
    const d = c < a;
    d && ([a, c] = [c, a]);
    const x = f && typeof f.range == "function" ? f : u(a, c, f),
      g = x ? x.range(a, +c + 1) : [];
    return d ? g.reverse() : g;
  }
  function u(a, c, f) {
    const d = Math.abs(c - a) / f,
      x = fa(([, , E]) => E).right(l, d);
    if (x === l.length) return e.every(iu(a / ql, c / ql, f));
    if (x === 0) return Bo.every(Math.max(iu(a, c, f), 1));
    const [g, v] = l[d / l[x - 1][2] < l[x][2] / d ? x - 1 : x];
    return g.every(v);
  }
  return [s, u];
}
const [bg, ex] = Jg($t, Ea, ml, Li, ka, wa);
function Jl(e) {
  if (0 <= e.y && e.y < 100) {
    var t = new Date(-1, e.m, e.d, e.H, e.M, e.S, e.L);
    return t.setFullYear(e.y), t;
  }
  return new Date(e.y, e.m, e.d, e.H, e.M, e.S, e.L);
}
function bl(e) {
  if (0 <= e.y && e.y < 100) {
    var t = new Date(Date.UTC(-1, e.m, e.d, e.H, e.M, e.S, e.L));
    return t.setUTCFullYear(e.y), t;
  }
  return new Date(Date.UTC(e.y, e.m, e.d, e.H, e.M, e.S, e.L));
}
function Nr(e, t, n) {
  return { y: e, m: t, d: n, H: 0, M: 0, S: 0, L: 0 };
}
function tx(e) {
  var t = e.dateTime,
    n = e.date,
    r = e.time,
    i = e.periods,
    o = e.days,
    l = e.shortDays,
    s = e.months,
    u = e.shortMonths,
    a = $r(i),
    c = Ar(i),
    f = $r(o),
    d = Ar(o),
    x = $r(l),
    g = Ar(l),
    v = $r(s),
    E = Ar(s),
    m = $r(u),
    p = Ar(u),
    y = {
      a: oe,
      A: T,
      b: L,
      B: z,
      c: null,
      d: vf,
      e: vf,
      f: Px,
      g: Dx,
      G: Ox,
      H: kx,
      I: Sx,
      j: Ex,
      L: gp,
      m: Cx,
      M: Mx,
      p: F,
      q: X,
      Q: Sf,
      s: Ef,
      S: jx,
      u: Tx,
      U: _x,
      V: Lx,
      w: Nx,
      W: $x,
      x: null,
      X: null,
      y: Ax,
      Y: zx,
      Z: Rx,
      "%": kf,
    },
    w = {
      a: Te,
      A: Ge,
      b: tn,
      B: vt,
      c: null,
      d: wf,
      e: wf,
      f: Hx,
      g: qx,
      G: bx,
      H: Fx,
      I: Ix,
      j: Ux,
      L: vp,
      m: Wx,
      M: Vx,
      p: Nn,
      q: t0,
      Q: Sf,
      s: Ef,
      S: Bx,
      u: Yx,
      U: Gx,
      V: Qx,
      w: Xx,
      W: Kx,
      x: null,
      X: null,
      y: Zx,
      Y: Jx,
      Z: e1,
      "%": kf,
    },
    k = {
      a: O,
      A: N,
      b: H,
      B: R,
      c: Y,
      d: gf,
      e: gf,
      f: gx,
      g: yf,
      G: mf,
      H: xf,
      I: xf,
      j: hx,
      L: yx,
      m: dx,
      M: px,
      p: D,
      q: fx,
      Q: vx,
      s: wx,
      S: mx,
      u: lx,
      U: sx,
      V: ux,
      w: ox,
      W: ax,
      x: K,
      X: Q,
      y: yf,
      Y: mf,
      Z: cx,
      "%": xx,
    };
  (y.x = S(n, y)),
    (y.X = S(r, y)),
    (y.c = S(t, y)),
    (w.x = S(n, w)),
    (w.X = S(r, w)),
    (w.c = S(t, w));
  function S(A, U) {
    return function (V) {
      var j = [],
        _e = -1,
        Z = 0,
        Oe = A.length,
        Re,
        nn,
        Da;
      for (V instanceof Date || (V = new Date(+V)); ++_e < Oe; )
        A.charCodeAt(_e) === 37 &&
          (j.push(A.slice(Z, _e)),
          (nn = pf[(Re = A.charAt(++_e))]) != null
            ? (Re = A.charAt(++_e))
            : (nn = Re === "e" ? " " : "0"),
          (Da = U[Re]) && (Re = Da(V, nn)),
          j.push(Re),
          (Z = _e + 1));
      return j.push(A.slice(Z, _e)), j.join("");
    };
  }
  function P(A, U) {
    return function (V) {
      var j = Nr(1900, void 0, 1),
        _e = C(j, A, (V += ""), 0),
        Z,
        Oe;
      if (_e != V.length) return null;
      if ("Q" in j) return new Date(j.Q);
      if ("s" in j) return new Date(j.s * 1e3 + ("L" in j ? j.L : 0));
      if (
        (U && !("Z" in j) && (j.Z = 0),
        "p" in j && (j.H = (j.H % 12) + j.p * 12),
        j.m === void 0 && (j.m = "q" in j ? j.q : 0),
        "V" in j)
      ) {
        if (j.V < 1 || j.V > 53) return null;
        "w" in j || (j.w = 1),
          "Z" in j
            ? ((Z = bl(Nr(j.y, 0, 1))),
              (Oe = Z.getUTCDay()),
              (Z = Oe > 4 || Oe === 0 ? Go.ceil(Z) : Go(Z)),
              (Z = Sa.offset(Z, (j.V - 1) * 7)),
              (j.y = Z.getUTCFullYear()),
              (j.m = Z.getUTCMonth()),
              (j.d = Z.getUTCDate() + ((j.w + 6) % 7)))
            : ((Z = Jl(Nr(j.y, 0, 1))),
              (Oe = Z.getDay()),
              (Z = Oe > 4 || Oe === 0 ? Yo.ceil(Z) : Yo(Z)),
              (Z = Li.offset(Z, (j.V - 1) * 7)),
              (j.y = Z.getFullYear()),
              (j.m = Z.getMonth()),
              (j.d = Z.getDate() + ((j.w + 6) % 7)));
      } else
        ("W" in j || "U" in j) &&
          ("w" in j || (j.w = "u" in j ? j.u % 7 : "W" in j ? 1 : 0),
          (Oe =
            "Z" in j
              ? bl(Nr(j.y, 0, 1)).getUTCDay()
              : Jl(Nr(j.y, 0, 1)).getDay()),
          (j.m = 0),
          (j.d =
            "W" in j
              ? ((j.w + 6) % 7) + j.W * 7 - ((Oe + 5) % 7)
              : j.w + j.U * 7 - ((Oe + 6) % 7)));
      return "Z" in j
        ? ((j.H += (j.Z / 100) | 0), (j.M += j.Z % 100), bl(j))
        : Jl(j);
    };
  }
  function C(A, U, V, j) {
    for (var _e = 0, Z = U.length, Oe = V.length, Re, nn; _e < Z; ) {
      if (j >= Oe) return -1;
      if (((Re = U.charCodeAt(_e++)), Re === 37)) {
        if (
          ((Re = U.charAt(_e++)),
          (nn = k[Re in pf ? U.charAt(_e++) : Re]),
          !nn || (j = nn(A, V, j)) < 0)
        )
          return -1;
      } else if (Re != V.charCodeAt(j++)) return -1;
    }
    return j;
  }
  function D(A, U, V) {
    var j = a.exec(U.slice(V));
    return j ? ((A.p = c.get(j[0].toLowerCase())), V + j[0].length) : -1;
  }
  function O(A, U, V) {
    var j = x.exec(U.slice(V));
    return j ? ((A.w = g.get(j[0].toLowerCase())), V + j[0].length) : -1;
  }
  function N(A, U, V) {
    var j = f.exec(U.slice(V));
    return j ? ((A.w = d.get(j[0].toLowerCase())), V + j[0].length) : -1;
  }
  function H(A, U, V) {
    var j = m.exec(U.slice(V));
    return j ? ((A.m = p.get(j[0].toLowerCase())), V + j[0].length) : -1;
  }
  function R(A, U, V) {
    var j = v.exec(U.slice(V));
    return j ? ((A.m = E.get(j[0].toLowerCase())), V + j[0].length) : -1;
  }
  function Y(A, U, V) {
    return C(A, t, U, V);
  }
  function K(A, U, V) {
    return C(A, n, U, V);
  }
  function Q(A, U, V) {
    return C(A, r, U, V);
  }
  function oe(A) {
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
  function Te(A) {
    return l[A.getUTCDay()];
  }
  function Ge(A) {
    return o[A.getUTCDay()];
  }
  function tn(A) {
    return u[A.getUTCMonth()];
  }
  function vt(A) {
    return s[A.getUTCMonth()];
  }
  function Nn(A) {
    return i[+(A.getUTCHours() >= 12)];
  }
  function t0(A) {
    return 1 + ~~(A.getUTCMonth() / 3);
  }
  return {
    format: function (A) {
      var U = S((A += ""), y);
      return (
        (U.toString = function () {
          return A;
        }),
        U
      );
    },
    parse: function (A) {
      var U = P((A += ""), !1);
      return (
        (U.toString = function () {
          return A;
        }),
        U
      );
    },
    utcFormat: function (A) {
      var U = S((A += ""), w);
      return (
        (U.toString = function () {
          return A;
        }),
        U
      );
    },
    utcParse: function (A) {
      var U = P((A += ""), !0);
      return (
        (U.toString = function () {
          return A;
        }),
        U
      );
    },
  };
}
var pf = { "-": "", _: " ", 0: "0" },
  me = /^\s*\d+/,
  nx = /^%/,
  rx = /[\\^$*+?|[\]().{}]/g;
function B(e, t, n) {
  var r = e < 0 ? "-" : "",
    i = (r ? -e : e) + "",
    o = i.length;
  return r + (o < n ? new Array(n - o + 1).join(t) + i : i);
}
function ix(e) {
  return e.replace(rx, "\\$&");
}
function $r(e) {
  return new RegExp("^(?:" + e.map(ix).join("|") + ")", "i");
}
function Ar(e) {
  return new Map(e.map((t, n) => [t.toLowerCase(), n]));
}
function ox(e, t, n) {
  var r = me.exec(t.slice(n, n + 1));
  return r ? ((e.w = +r[0]), n + r[0].length) : -1;
}
function lx(e, t, n) {
  var r = me.exec(t.slice(n, n + 1));
  return r ? ((e.u = +r[0]), n + r[0].length) : -1;
}
function sx(e, t, n) {
  var r = me.exec(t.slice(n, n + 2));
  return r ? ((e.U = +r[0]), n + r[0].length) : -1;
}
function ux(e, t, n) {
  var r = me.exec(t.slice(n, n + 2));
  return r ? ((e.V = +r[0]), n + r[0].length) : -1;
}
function ax(e, t, n) {
  var r = me.exec(t.slice(n, n + 2));
  return r ? ((e.W = +r[0]), n + r[0].length) : -1;
}
function mf(e, t, n) {
  var r = me.exec(t.slice(n, n + 4));
  return r ? ((e.y = +r[0]), n + r[0].length) : -1;
}
function yf(e, t, n) {
  var r = me.exec(t.slice(n, n + 2));
  return r ? ((e.y = +r[0] + (+r[0] > 68 ? 1900 : 2e3)), n + r[0].length) : -1;
}
function cx(e, t, n) {
  var r = /^(Z)|([+-]\d\d)(?::?(\d\d))?/.exec(t.slice(n, n + 6));
  return r
    ? ((e.Z = r[1] ? 0 : -(r[2] + (r[3] || "00"))), n + r[0].length)
    : -1;
}
function fx(e, t, n) {
  var r = me.exec(t.slice(n, n + 1));
  return r ? ((e.q = r[0] * 3 - 3), n + r[0].length) : -1;
}
function dx(e, t, n) {
  var r = me.exec(t.slice(n, n + 2));
  return r ? ((e.m = r[0] - 1), n + r[0].length) : -1;
}
function gf(e, t, n) {
  var r = me.exec(t.slice(n, n + 2));
  return r ? ((e.d = +r[0]), n + r[0].length) : -1;
}
function hx(e, t, n) {
  var r = me.exec(t.slice(n, n + 3));
  return r ? ((e.m = 0), (e.d = +r[0]), n + r[0].length) : -1;
}
function xf(e, t, n) {
  var r = me.exec(t.slice(n, n + 2));
  return r ? ((e.H = +r[0]), n + r[0].length) : -1;
}
function px(e, t, n) {
  var r = me.exec(t.slice(n, n + 2));
  return r ? ((e.M = +r[0]), n + r[0].length) : -1;
}
function mx(e, t, n) {
  var r = me.exec(t.slice(n, n + 2));
  return r ? ((e.S = +r[0]), n + r[0].length) : -1;
}
function yx(e, t, n) {
  var r = me.exec(t.slice(n, n + 3));
  return r ? ((e.L = +r[0]), n + r[0].length) : -1;
}
function gx(e, t, n) {
  var r = me.exec(t.slice(n, n + 6));
  return r ? ((e.L = Math.floor(r[0] / 1e3)), n + r[0].length) : -1;
}
function xx(e, t, n) {
  var r = nx.exec(t.slice(n, n + 1));
  return r ? n + r[0].length : -1;
}
function vx(e, t, n) {
  var r = me.exec(t.slice(n));
  return r ? ((e.Q = +r[0]), n + r[0].length) : -1;
}
function wx(e, t, n) {
  var r = me.exec(t.slice(n));
  return r ? ((e.s = +r[0]), n + r[0].length) : -1;
}
function vf(e, t) {
  return B(e.getDate(), t, 2);
}
function kx(e, t) {
  return B(e.getHours(), t, 2);
}
function Sx(e, t) {
  return B(e.getHours() % 12 || 12, t, 2);
}
function Ex(e, t) {
  return B(1 + Li.count($t(e), e), t, 3);
}
function gp(e, t) {
  return B(e.getMilliseconds(), t, 3);
}
function Px(e, t) {
  return gp(e, t) + "000";
}
function Cx(e, t) {
  return B(e.getMonth() + 1, t, 2);
}
function Mx(e, t) {
  return B(e.getMinutes(), t, 2);
}
function jx(e, t) {
  return B(e.getSeconds(), t, 2);
}
function Tx(e) {
  var t = e.getDay();
  return t === 0 ? 7 : t;
}
function _x(e, t) {
  return B(ml.count($t(e) - 1, e), t, 2);
}
function xp(e) {
  var t = e.getDay();
  return t >= 4 || t === 0 ? hr(e) : hr.ceil(e);
}
function Lx(e, t) {
  return (e = xp(e)), B(hr.count($t(e), e) + ($t(e).getDay() === 4), t, 2);
}
function Nx(e) {
  return e.getDay();
}
function $x(e, t) {
  return B(Yo.count($t(e) - 1, e), t, 2);
}
function Ax(e, t) {
  return B(e.getFullYear() % 100, t, 2);
}
function Dx(e, t) {
  return (e = xp(e)), B(e.getFullYear() % 100, t, 2);
}
function zx(e, t) {
  return B(e.getFullYear() % 1e4, t, 4);
}
function Ox(e, t) {
  var n = e.getDay();
  return (
    (e = n >= 4 || n === 0 ? hr(e) : hr.ceil(e)), B(e.getFullYear() % 1e4, t, 4)
  );
}
function Rx(e) {
  var t = e.getTimezoneOffset();
  return (
    (t > 0 ? "-" : ((t *= -1), "+")) +
    B((t / 60) | 0, "0", 2) +
    B(t % 60, "0", 2)
  );
}
function wf(e, t) {
  return B(e.getUTCDate(), t, 2);
}
function Fx(e, t) {
  return B(e.getUTCHours(), t, 2);
}
function Ix(e, t) {
  return B(e.getUTCHours() % 12 || 12, t, 2);
}
function Ux(e, t) {
  return B(1 + Sa.count(En(e), e), t, 3);
}
function vp(e, t) {
  return B(e.getUTCMilliseconds(), t, 3);
}
function Hx(e, t) {
  return vp(e, t) + "000";
}
function Wx(e, t) {
  return B(e.getUTCMonth() + 1, t, 2);
}
function Vx(e, t) {
  return B(e.getUTCMinutes(), t, 2);
}
function Bx(e, t) {
  return B(e.getUTCSeconds(), t, 2);
}
function Yx(e) {
  var t = e.getUTCDay();
  return t === 0 ? 7 : t;
}
function Gx(e, t) {
  return B(yp.count(En(e) - 1, e), t, 2);
}
function wp(e) {
  var t = e.getUTCDay();
  return t >= 4 || t === 0 ? pr(e) : pr.ceil(e);
}
function Qx(e, t) {
  return (e = wp(e)), B(pr.count(En(e), e) + (En(e).getUTCDay() === 4), t, 2);
}
function Xx(e) {
  return e.getUTCDay();
}
function Kx(e, t) {
  return B(Go.count(En(e) - 1, e), t, 2);
}
function Zx(e, t) {
  return B(e.getUTCFullYear() % 100, t, 2);
}
function qx(e, t) {
  return (e = wp(e)), B(e.getUTCFullYear() % 100, t, 2);
}
function Jx(e, t) {
  return B(e.getUTCFullYear() % 1e4, t, 4);
}
function bx(e, t) {
  var n = e.getUTCDay();
  return (
    (e = n >= 4 || n === 0 ? pr(e) : pr.ceil(e)),
    B(e.getUTCFullYear() % 1e4, t, 4)
  );
}
function e1() {
  return "+0000";
}
function kf() {
  return "%";
}
function Sf(e) {
  return +e;
}
function Ef(e) {
  return Math.floor(+e / 1e3);
}
var Dn, kp;
t1({
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
function t1(e) {
  return (
    (Dn = tx(e)), (kp = Dn.format), Dn.parse, Dn.utcFormat, Dn.utcParse, Dn
  );
}
function n1(e) {
  return new Date(e);
}
function r1(e) {
  return e instanceof Date ? +e : +new Date(+e);
}
function Sp(e, t, n, r, i, o, l, s, u, a) {
  var c = cp(),
    f = c.invert,
    d = c.domain,
    x = a(".%L"),
    g = a(":%S"),
    v = a("%I:%M"),
    E = a("%I %p"),
    m = a("%a %d"),
    p = a("%b %d"),
    y = a("%B"),
    w = a("%Y");
  function k(S) {
    return (
      u(S) < S
        ? x
        : s(S) < S
        ? g
        : l(S) < S
        ? v
        : o(S) < S
        ? E
        : r(S) < S
        ? i(S) < S
          ? m
          : p
        : n(S) < S
        ? y
        : w
    )(S);
  }
  return (
    (c.invert = function (S) {
      return new Date(f(S));
    }),
    (c.domain = function (S) {
      return arguments.length ? d(Array.from(S, r1)) : d().map(n1);
    }),
    (c.ticks = function (S) {
      var P = d();
      return e(P[0], P[P.length - 1], S ?? 10);
    }),
    (c.tickFormat = function (S, P) {
      return P == null ? k : a(P);
    }),
    (c.nice = function (S) {
      var P = d();
      return (
        (!S || typeof S.range != "function") &&
          (S = t(P[0], P[P.length - 1], S ?? 10)),
        S ? d(pp(P, S)) : c
      );
    }),
    (c.copy = function () {
      return ya(c, Sp(e, t, n, r, i, o, l, s, u, a));
    }),
    c
  );
}
function i1() {
  return pl.apply(
    Sp(bg, ex, $t, Ea, ml, Li, ka, wa, Zn, kp).domain([
      new Date(2e3, 0, 1),
      new Date(2e3, 0, 2),
    ]),
    arguments
  );
}
function o1(e) {
  for (var t = (e.length / 6) | 0, n = new Array(t), r = 0; r < t; )
    n[r] = "#" + e.slice(r * 6, ++r * 6);
  return n;
}
const l1 = o1("e41a1c377eb84daf4a984ea3ff7f00ffff33a65628f781bf999999");
function st(e) {
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
function mr(e) {
  return !!e && !!e[We];
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
        (typeof i == "function" && Function.toString.call(i) === p1)
      );
    })(e) ||
      Array.isArray(e) ||
      !!e[Nf] ||
      !!(!((t = e.constructor) === null || t === void 0) && t[Nf]) ||
      Pa(e) ||
      Ca(e))
  );
}
function ki(e, t, n) {
  n === void 0 && (n = !1),
    vr(e) === 0
      ? (n ? Object.keys : La)(e).forEach(function (r) {
          (n && typeof r == "symbol") || t(r, e[r], e);
        })
      : e.forEach(function (r, i) {
          return t(i, r, e);
        });
}
function vr(e) {
  var t = e[We];
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
function uu(e, t) {
  return vr(e) === 2 ? e.has(t) : Object.prototype.hasOwnProperty.call(e, t);
}
function s1(e, t) {
  return vr(e) === 2 ? e.get(t) : e[t];
}
function Ep(e, t, n) {
  var r = vr(e);
  r === 2 ? e.set(t, n) : r === 3 ? e.add(n) : (e[t] = n);
}
function u1(e, t) {
  return e === t ? e !== 0 || 1 / e == 1 / t : e != e && t != t;
}
function Pa(e) {
  return d1 && e instanceof Map;
}
function Ca(e) {
  return h1 && e instanceof Set;
}
function ln(e) {
  return e.o || e.t;
}
function Ma(e) {
  if (Array.isArray(e)) return Array.prototype.slice.call(e);
  var t = m1(e);
  delete t[We];
  for (var n = La(t), r = 0; r < n.length; r++) {
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
function ja(e, t) {
  return (
    t === void 0 && (t = !1),
    Ta(e) ||
      mr(e) ||
      !Pn(e) ||
      (vr(e) > 1 && (e.set = e.add = e.clear = e.delete = a1),
      Object.freeze(e),
      t &&
        ki(
          e,
          function (n, r) {
            return ja(r, !0);
          },
          !0
        )),
    e
  );
}
function a1() {
  st(2);
}
function Ta(e) {
  return e == null || typeof e != "object" || Object.isFrozen(e);
}
function xt(e) {
  var t = y1[e];
  return t || st(18, e), t;
}
function Pf() {
  return Si;
}
function es(e, t) {
  t && (xt("Patches"), (e.u = []), (e.s = []), (e.v = t));
}
function Qo(e) {
  au(e), e.p.forEach(c1), (e.p = null);
}
function au(e) {
  e === Si && (Si = e.l);
}
function Cf(e) {
  return (Si = { p: [], l: Si, h: e, m: !0, _: 0 });
}
function c1(e) {
  var t = e[We];
  t.i === 0 || t.i === 1 ? t.j() : (t.g = !0);
}
function ts(e, t) {
  t._ = t.p.length;
  var n = t.p[0],
    r = e !== void 0 && e !== n;
  return (
    t.h.O || xt("ES5").S(t, e, r),
    r
      ? (n[We].P && (Qo(t), st(4)),
        Pn(e) && ((e = Xo(t, e)), t.l || Ko(t, e)),
        t.u && xt("Patches").M(n[We].t, e, t.u, t.s))
      : (e = Xo(t, n, [])),
    Qo(t),
    t.u && t.v(t.u, t.s),
    e !== Pp ? e : void 0
  );
}
function Xo(e, t, n) {
  if (Ta(t)) return t;
  var r = t[We];
  if (!r)
    return (
      ki(
        t,
        function (s, u) {
          return Mf(e, r, t, s, u, n);
        },
        !0
      ),
      t
    );
  if (r.A !== e) return t;
  if (!r.P) return Ko(e, r.t, !0), r.t;
  if (!r.I) {
    (r.I = !0), r.A._--;
    var i = r.i === 4 || r.i === 5 ? (r.o = Ma(r.k)) : r.o,
      o = i,
      l = !1;
    r.i === 3 && ((o = new Set(i)), i.clear(), (l = !0)),
      ki(o, function (s, u) {
        return Mf(e, r, i, s, u, n, l);
      }),
      Ko(e, i, !1),
      n && e.u && xt("Patches").N(r, n, e.u, e.s);
  }
  return r.o;
}
function Mf(e, t, n, r, i, o, l) {
  if (mr(i)) {
    var s = Xo(e, i, o && t && t.i !== 3 && !uu(t.R, r) ? o.concat(r) : void 0);
    if ((Ep(n, r, s), !mr(s))) return;
    e.m = !1;
  } else l && n.add(i);
  if (Pn(i) && !Ta(i)) {
    if (!e.h.D && e._ < 1) return;
    Xo(e, i), (t && t.A.l) || Ko(e, i);
  }
}
function Ko(e, t, n) {
  n === void 0 && (n = !1), !e.l && e.h.D && e.m && ja(t, n);
}
function ns(e, t) {
  var n = e[We];
  return (n ? ln(n) : e)[t];
}
function jf(e, t) {
  if (t in e)
    for (var n = Object.getPrototypeOf(e); n; ) {
      var r = Object.getOwnPropertyDescriptor(n, t);
      if (r) return r;
      n = Object.getPrototypeOf(n);
    }
}
function cu(e) {
  e.P || ((e.P = !0), e.l && cu(e.l));
}
function rs(e) {
  e.o || (e.o = Ma(e.t));
}
function fu(e, t, n) {
  var r = Pa(t)
    ? xt("MapSet").F(t, n)
    : Ca(t)
    ? xt("MapSet").T(t, n)
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
          a = du;
        l && ((u = [s]), (a = Ur));
        var c = Proxy.revocable(u, a),
          f = c.revoke,
          d = c.proxy;
        return (s.k = d), (s.j = f), d;
      })(t, n)
    : xt("ES5").J(t, n);
  return (n ? n.A : Pf()).p.push(r), r;
}
function f1(e) {
  return (
    mr(e) || st(22, e),
    (function t(n) {
      if (!Pn(n)) return n;
      var r,
        i = n[We],
        o = vr(n);
      if (i) {
        if (!i.P && (i.i < 4 || !xt("ES5").K(i))) return i.t;
        (i.I = !0), (r = Tf(n, o)), (i.I = !1);
      } else r = Tf(n, o);
      return (
        ki(r, function (l, s) {
          (i && s1(i.t, l) === s) || Ep(r, l, t(s));
        }),
        o === 3 ? new Set(r) : r
      );
    })(e)
  );
}
function Tf(e, t) {
  switch (t) {
    case 2:
      return new Map(e);
    case 3:
      return Array.from(e);
  }
  return Ma(e);
}
var _f,
  Si,
  _a = typeof Symbol < "u" && typeof Symbol("x") == "symbol",
  d1 = typeof Map < "u",
  h1 = typeof Set < "u",
  Lf = typeof Proxy < "u" && Proxy.revocable !== void 0 && typeof Reflect < "u",
  Pp = _a
    ? Symbol.for("immer-nothing")
    : (((_f = {})["immer-nothing"] = !0), _f),
  Nf = _a ? Symbol.for("immer-draftable") : "__$immer_draftable",
  We = _a ? Symbol.for("immer-state") : "__$immer_state",
  p1 = "" + Object.prototype.constructor,
  La =
    typeof Reflect < "u" && Reflect.ownKeys
      ? Reflect.ownKeys
      : Object.getOwnPropertySymbols !== void 0
      ? function (e) {
          return Object.getOwnPropertyNames(e).concat(
            Object.getOwnPropertySymbols(e)
          );
        }
      : Object.getOwnPropertyNames,
  m1 =
    Object.getOwnPropertyDescriptors ||
    function (e) {
      var t = {};
      return (
        La(e).forEach(function (n) {
          t[n] = Object.getOwnPropertyDescriptor(e, n);
        }),
        t
      );
    },
  y1 = {},
  du = {
    get: function (e, t) {
      if (t === We) return e;
      var n = ln(e);
      if (!uu(n, t))
        return (function (i, o, l) {
          var s,
            u = jf(o, l);
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
        : r === ns(e.t, t)
        ? (rs(e), (e.o[t] = fu(e.A.h, r, e)))
        : r;
    },
    has: function (e, t) {
      return t in ln(e);
    },
    ownKeys: function (e) {
      return Reflect.ownKeys(ln(e));
    },
    set: function (e, t, n) {
      var r = jf(ln(e), t);
      if (r != null && r.set) return r.set.call(e.k, n), !0;
      if (!e.P) {
        var i = ns(ln(e), t),
          o = i == null ? void 0 : i[We];
        if (o && o.t === n) return (e.o[t] = n), (e.R[t] = !1), !0;
        if (u1(n, i) && (n !== void 0 || uu(e.t, t))) return !0;
        rs(e), cu(e);
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
        ns(e.t, t) !== void 0 || t in e.t
          ? ((e.R[t] = !1), rs(e), cu(e))
          : delete e.R[t],
        e.o && delete e.o[t],
        !0
      );
    },
    getOwnPropertyDescriptor: function (e, t) {
      var n = ln(e),
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
      st(11);
    },
    getPrototypeOf: function (e) {
      return Object.getPrototypeOf(e.t);
    },
    setPrototypeOf: function () {
      st(12);
    },
  },
  Ur = {};
ki(du, function (e, t) {
  Ur[e] = function () {
    return (arguments[0] = arguments[0][0]), t.apply(this, arguments);
  };
}),
  (Ur.deleteProperty = function (e, t) {
    return Ur.set.call(this, e, t, void 0);
  }),
  (Ur.set = function (e, t, n) {
    return du.set.call(this, e[0], t, n, e[0]);
  });
var g1 = (function () {
    function e(n) {
      var r = this;
      (this.O = Lf),
        (this.D = !0),
        (this.produce = function (i, o, l) {
          if (typeof i == "function" && typeof o != "function") {
            var s = o;
            o = i;
            var u = r;
            return function (v) {
              var E = this;
              v === void 0 && (v = s);
              for (
                var m = arguments.length, p = Array(m > 1 ? m - 1 : 0), y = 1;
                y < m;
                y++
              )
                p[y - 1] = arguments[y];
              return u.produce(v, function (w) {
                var k;
                return (k = o).call.apply(k, [E, w].concat(p));
              });
            };
          }
          var a;
          if (
            (typeof o != "function" && st(6),
            l !== void 0 && typeof l != "function" && st(7),
            Pn(i))
          ) {
            var c = Cf(r),
              f = fu(r, i, void 0),
              d = !0;
            try {
              (a = o(f)), (d = !1);
            } finally {
              d ? Qo(c) : au(c);
            }
            return typeof Promise < "u" && a instanceof Promise
              ? a.then(
                  function (v) {
                    return es(c, l), ts(v, c);
                  },
                  function (v) {
                    throw (Qo(c), v);
                  }
                )
              : (es(c, l), ts(a, c));
          }
          if (!i || typeof i != "object") {
            if (
              ((a = o(i)) === void 0 && (a = i),
              a === Pp && (a = void 0),
              r.D && ja(a, !0),
              l)
            ) {
              var x = [],
                g = [];
              xt("Patches").M(i, a, x, g), l(x, g);
            }
            return a;
          }
          st(21, i);
        }),
        (this.produceWithPatches = function (i, o) {
          if (typeof i == "function")
            return function (a) {
              for (
                var c = arguments.length, f = Array(c > 1 ? c - 1 : 0), d = 1;
                d < c;
                d++
              )
                f[d - 1] = arguments[d];
              return r.produceWithPatches(a, function (x) {
                return i.apply(void 0, [x].concat(f));
              });
            };
          var l,
            s,
            u = r.produce(i, o, function (a, c) {
              (l = a), (s = c);
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
        Pn(n) || st(8), mr(n) && (n = f1(n));
        var r = Cf(this),
          i = fu(this, n, void 0);
        return (i[We].C = !0), au(r), i;
      }),
      (t.finishDraft = function (n, r) {
        var i = n && n[We],
          o = i.A;
        return es(o, r), ts(void 0, o);
      }),
      (t.setAutoFreeze = function (n) {
        this.D = n;
      }),
      (t.setUseProxies = function (n) {
        n && !Lf && st(20), (this.O = n);
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
        var l = xt("Patches").$;
        return mr(n)
          ? l(n, r)
          : this.produce(n, function (s) {
              return l(s, r);
            });
      }),
      e
    );
  })(),
  Ve = new g1(),
  Cp = Ve.produce;
Ve.produceWithPatches.bind(Ve);
Ve.setAutoFreeze.bind(Ve);
Ve.setUseProxies.bind(Ve);
Ve.applyPatches.bind(Ve);
Ve.createDraft.bind(Ve);
Ve.finishDraft.bind(Ve);
const $f = { x: 0, y: 0, width: 0, height: 0 };
function ht() {
  const [e, t] = _.useState($f),
    n = _.useRef($f),
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
            const c = { x: u.x, y: u.y, width: u.width, height: u.height };
            (n.current = c), t(c);
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
function Af(e, t, n, r) {
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
function Ni(e) {
  const {
      x: t = 0,
      y: n = 0,
      verticalAlign: r = "start",
      horizontalAlign: i = "start",
      children: o,
      style: l = {},
    } = e,
    s = ht(),
    u = _.useMemo(() => Af(t - s.x, i, s.width || 0, t), [t, s.x, s.width, i]),
    a = _.useMemo(
      () => Af(n - s.y, r, s.height || 0, n),
      [n, s.y, s.height, r]
    );
  return h.jsx("g", {
    style: l,
    ref: s.ref,
    transform: `translate(${u}, ${a})`,
    children: o,
  });
}
function Zo(e, t) {
  var n, r;
  const i = document.createTextNode(e),
    o = document.createElementNS("http://www.w3.org/2000/svg", "text");
  o.setAttribute("class", "test"),
    o.appendChild(i),
    (n = t.current) === null || n === void 0 || n.appendChild(o);
  const l = o.getBBox();
  return (r = t.current) === null || r === void 0 || r.removeChild(o), l;
}
const x1 = "+1234567890";
function Mp(e, t, n, r) {
  const i = e.range();
  if (!i) throw new Error("Range needs to be specified");
  if (!e.domain()) throw new Error("Domain needs to be specified");
  const { minSpace: l = 8, tickFormat: s, setTicks: u } = r,
    a = Math.abs(i[0] - i[1]);
  _.useEffect(() => {
    if (n.current) {
      let c,
        f = [];
      if (t === "horizontal")
        for (;;) {
          f = e.ticks(c);
          const d = f.map(s);
          c = Math.min(f.length, c || 1 / 0);
          const { width: x } = Zo(d.join(""), n);
          if (x + (f.length - 1) * l > a && c > 1) c = c - 1;
          else break;
        }
      else {
        const { height: d } = Zo(x1, n);
        for (
          ;
          (f = e.ticks(c)),
            (c = Math.min(f.length, c || 1 / 0)),
            f.length * (d + l) - l > a && c > 1;

        )
          c = c - 1;
      }
      u(f.map((d) => ({ label: s(d), position: e(d), value: d })));
    }
  }, [a, t, l, n, e, u, s]);
}
function v1(e, t, n, r = {}) {
  const [i, o] = _.useState([]),
    { tickFormat: l = String } = r;
  return Mp(e, t, n, { ...r, tickFormat: l, setTicks: o }), i;
}
const w1 = "+1234567890";
function hu(e) {
  const t = e / Math.pow(10, Math.round(Math.log10(e)));
  return Math.floor(t < 1 ? t * 10 : t);
}
function k1(e, t, n, r, i) {
  const o = e.filter((c) => hu(c) === 1).map(t),
    l = Math.abs(o[0] - o[1]),
    s = (r + i) / l,
    u = s >= 1 ? Math.ceil(s) : 1;
  let a = 0;
  return e.map((c) => {
    const f = t(c);
    let d = "";
    return (
      hu(c) === 1 && ((d = a === 0 ? n(c) : ""), (a = (a + 1) % u)),
      { label: d, position: f, value: c }
    );
  });
}
function S1(e, t, n, r = {}) {
  const [i, o] = _.useState(40);
  if (!e.range()) throw new Error("Range needs to be specified");
  const s = e.domain();
  if (!s) throw new Error("Domain needs to be specified");
  const { minSpace: u = 8 } = r,
    a = r == null ? void 0 : r.tickFormat,
    c = _.useCallback((d) => (a ? a(d) : String(d)), [a]),
    f = _.useMemo(() => e.ticks(), [e]);
  return (
    _.useEffect(() => {
      if (n.current)
        if (t === "horizontal") {
          const d = f
              .filter((g) => hu(g) === 1)
              .map(c)
              .reduce((g, v) => (g.length < v.length ? v : g), ""),
            { width: x } = Zo(d, n);
          o(Math.ceil(x));
        } else {
          const { height: d } = Zo(w1, n);
          o(Math.ceil(d));
        }
    }, [t, s, c, n, f]),
    k1(f, e, c, i, u)
  );
}
function E1(e, t, n, r) {
  const { tickFormat: i = e.tickFormat() } = r,
    [o, l] = _.useState([]);
  return Mp(e, t, n, { ...r, setTicks: l, tickFormat: i }), o;
}
const jp = _.createContext(null);
function wr() {
  const e = _.useContext(jp);
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
    return h.jsx(jp.Provider, { value: r, children: e.children });
  };
let j1 = 1;
function T1() {
  return ++j1;
}
function kr(e, t) {
  return _.useMemo(() => e || `${t}-${T1()}`, [e, t]);
}
const un = ["top", "bottom"],
  Rt = ["left", "right"];
function _1(e, t, n) {
  if (
    (un.includes(e) && !un.includes(t)) ||
    (Rt.includes(e) && !Rt.includes(t))
  )
    throw new Error(`The positions are ortogonal for ${n}`);
}
function tt(e, t, n, r = {}) {
  const { onlyOrthogonal: i = !1 } = r,
    o = e[t],
    l = e[n];
  if (!o || !l) return [void 0, void 0];
  if (
    i &&
    ((un.includes(o.position) && un.includes(l.position)) ||
      (Rt.includes(l.position) && Rt.includes(o.position)))
  )
    throw new Error(`The axis ${t} and ${n} are not orthogonal`);
  if (
    !i &&
    (un.includes(o.position)
      ? !Rt.includes(l.position)
      : Rt.includes(o.position))
  )
    throw Rt.includes(o.position) || un.includes(l.position)
      ? new Error(
          `The axis ${t} should be ${un.join(" ")} and ${n} should be ${Rt.join(
            " "
          )}`
        )
      : new Error(`The axis ${t} and ${n} are not orthogonal`);
  return [o.scale, l.scale];
}
function yn(e, t, n, r, i) {
  let o = { ...e };
  for (const l in t)
    typeof t[l] == "function" ? (o[l] = t[l](n, r, i)) : (o[l] = t[l]);
  return o;
}
function L1(e, t, n, r) {
  let i;
  return typeof e == "function" ? (i = e(t, n, r)) : (i = e), i;
}
function N1(e, t, n, r) {
  let i;
  return typeof e == "function" ? (i = e(t, n, r)) : (i = e), i;
}
function Df(e) {
  return typeof e == "number"
    ? [e, e]
    : Array.isArray(e) && e.length >= 2
    ? e
    : null;
}
function is(e, t, n) {
  let r = { index: 0, distance: Number.POSITIVE_INFINITY };
  for (let i = 0; i < e.length; i++) {
    const o = n(e[i], t);
    o < r.distance && ((r.index = i), (r.distance = o));
  }
  return e[r.index];
}
function Hr(e) {
  return typeof e > "u" || typeof e == "number" ? e : e.getTime();
}
function zf(e, t) {
  return { x: (e.x + t.x) / 2, y: (e.y + t.y) / 2 };
}
function $1(e, t) {
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
        ? (_1(o.position, r, n), (e.axes[n] = { ...o, position: r, ...i }))
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
const Tp = _.createContext({
    width: 0,
    height: 0,
    plotWidth: 0,
    plotHeight: 0,
    colorScaler: da(),
    axisContext: {},
  }),
  _p = _.createContext(() => {});
function se() {
  const e = _.useContext(Tp);
  if (!e) throw new Error("usePlotContext called outside of Plot context");
  return e;
}
function $i() {
  const e = _.useContext(_p);
  if (!e)
    throw new Error("usePlotDispatchContext called outside of Plot context");
  return e;
}
function A1(e, t, { plotWidth: n, plotHeight: r }) {
  return _.useMemo(() => {
    const o = {};
    for (const l in e.axes) {
      const s = e.axes[l],
        u = t[l],
        a = ["top", "bottom"].includes(s.position),
        c = a ? "x" : "y";
      let f = !1,
        d;
      (u == null ? void 0 : u.min) != null
        ? ((d = u.min), (f = !0))
        : s.min != null
        ? ((d = s.min), (f = !0))
        : (d = Yy(
            e.series.filter((w) => c !== "x" || !w.id.startsWith("~")),
            (w) => (w[c].axisId === l ? w[c].min : void 0)
          ));
      let x = !1,
        g;
      if (
        ((u == null ? void 0 : u.max) != null
          ? ((g = u.max), (x = !0))
          : s.max != null
          ? ((g = s.max), (x = !0))
          : (g = By(
              e.series.filter((w) => c !== "x" || !w.id.startsWith("~")),
              (w) => (w[c].axisId === l ? w[c].max : void 0)
            )),
        d === void 0 || g === void 0)
      )
        return {};
      if (d > g) throw new Error(`${l}: min (${d}) is bigger than max (${g})`);
      const v = a ? n : r,
        E = D1(s, g - d, v, f, x),
        m = a ? [0, n] : [r, 0],
        p = [d - E.min, g + E.max],
        y = function (k) {
          return k < p[0] ? p[0] : k > p[1] ? p[1] : k;
        };
      switch (s.scale) {
        case "log": {
          o[l] = {
            type: s.scale,
            position: s.position,
            tickLabelFormat: s.tickLabelFormat,
            scale: mp()
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
            scale: i1()
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
            scale: hp()
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
function D1(e, t, n, r, i) {
  const { paddingStart: o, paddingEnd: l } = e;
  if (r && i) return { min: 0, max: 0 };
  if (i) return { min: os(o, 0, t, n).start, max: 0 };
  if (r) return { min: 0, max: os(0, l, t, n).end };
  {
    const s = os(o, l, t, n);
    return { min: s.start, max: s.end };
  }
}
function os(e, t, n, r) {
  let i = 0,
    o = 0,
    l = n;
  typeof e == "number" && ((l += e), (i = e)),
    typeof t == "number" && ((l += t), (o = t));
  let s = 0,
    u = 0;
  typeof e == "string" && (s = Of(e, r) / r),
    typeof t == "string" && (u = Of(t, r) / r);
  const a = s + u;
  if (a !== 0) {
    const c = (a * l) / (1 - a);
    (i = (s / a) * c), (o = (u / a) * c);
  }
  return { start: i, end: o };
}
function Of(e, t) {
  return e.endsWith("%") ? (Number(e.slice(0, -1)) / 100) * t : Number(e);
}
function yl(e) {
  const t = _.createContext(new Map());
  function n(i) {
    const o = _.useContext(t);
    return o.has(i) ? o.get(i) : e;
  }
  function r(i) {
    const { id: o, value: l, children: s } = i,
      u = _.useContext(t),
      a = _.useMemo(() => {
        const c = new Map(u);
        return c.set(o, l), c;
      }, [u, o, l]);
    return h.jsx(t.Provider, { value: a, children: s });
  }
  return { useNestedContext: n, NestedContextProvider: r };
}
const z1 = { axes: {} },
  O1 = yl(z1);
function R1(e) {
  return O1.useNestedContext(e == null ? void 0 : e.controllerId);
}
yl(null);
yl(null);
const F1 = yl(null);
function I1(e) {
  return F1.useNestedContext(e == null ? void 0 : e.controllerId);
}
function U1(e, t) {
  let n = 0;
  for (let r = 0; r < e.length; r++) n += (e[r] - t[r]) * (e[r] - t[r]);
  return n;
}
function Wr(e, t) {
  return Math.sqrt(U1(e, t));
}
function Cn(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = se(),
    { x: i, y: o, xAxis: l, yAxis: s } = e,
    [u, a] = tt(t, l, s, { onlyOrthogonal: !0 });
  return { x: Se(i, n, u), y: Se(o, r, a) };
}
function Lp(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = se(),
    { points: i, xAxis: o, yAxis: l } = e,
    [s, u] = tt(t, o, l, { onlyOrthogonal: !0 });
  return i.map((a) => `${Se(a.x, n, s)},${Se(a.y, r, u)}`).join(" ");
}
function H1(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = se(),
    { x1: i, y1: o, x2: l, y2: s, xAxis: u, yAxis: a } = e,
    [c, f] = tt(t, u, a, { onlyOrthogonal: !0 });
  return {
    x: Rf(i, l, n, c),
    y: Rf(o, s, r, f),
    width: If(i, l, n, c),
    height: If(o, s, r, f),
  };
}
function Np(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = se(),
    { cx: i, cy: o, rx: l, ry: s, xAxis: u, yAxis: a } = e,
    [c, f] = tt(t, u, a, { onlyOrthogonal: !0 });
  return { cx: Se(i, n, c), cy: Se(o, r, f), rx: Ei(l, n, c), ry: Ei(s, r, f) };
}
function W1(e) {
  var t;
  const { axisContext: n, plotWidth: r, plotHeight: i } = se(),
    {
      min: o,
      max: l,
      q1: s,
      median: u,
      q3: a,
      width: c,
      y: f,
      xAxis: d,
      yAxis: x,
    } = e,
    [g, v] = tt(n, d, x, { onlyOrthogonal: !0 }),
    E = ["top", "bottom"].includes(
      (t = n[d]) === null || t === void 0 ? void 0 : t.position
    );
  return {
    min: Se(o, r, g),
    max: Se(l, r, g),
    q1: Se(s, r, g),
    median: Se(u, r, g),
    q3: Se(a, r, g),
    y: Se(f, i, v),
    width: Ei(c, i, v),
    horizontal: E,
  };
}
function V1(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = se(),
    { x1: i, y1: o, x2: l, y2: s, width: u, xAxis: a, yAxis: c } = e,
    [f, d] = tt(t, a, c, { onlyOrthogonal: !0 }),
    {
      x1: x,
      y1: g,
      x2: v,
      y2: E,
    } = { x1: Se(i, n, f), x2: Se(l, n, f), y1: Se(o, n, d), y2: Se(s, n, d) },
    { cx: m, cy: p } = { cx: (x + v) / 2, cy: (g + E) / 2 },
    y =
      (g > E ? -1 : 1) *
      (x > v ? -1 : 1) *
      Math.asin(Wr([x, g], [x, p]) / Wr([x, g], [m, p])),
    { widthX: w, widthY: k } = {
      widthX: (Math.sin(y) * Ei(u, r, f)) / 2,
      widthY: (Math.cos(y) * Ei(u, r, d)) / 2,
    };
  return {
    cx: m,
    cy: p,
    rx: Wr([x, g], [v, E]) / 2,
    ry: Wr([0, 0], [w, k]),
    rotation: B1(y),
  };
}
function B1(e) {
  return (e * 180) / Math.PI;
}
function Mn(e, t) {
  return e.endsWith("%") ? (Number(e.slice(0, -1)) * t) / 100 : Number(e);
}
function Se(e, t, n) {
  return n === void 0 ? 0 : typeof e == "number" ? n(e) : Mn(e, t);
}
function Rf(e, t, n, r) {
  return r === void 0
    ? 0
    : Math.min(
        typeof t == "number" ? r(t) : Mn(t, n),
        typeof e == "number" ? r(e) : Mn(e, n)
      );
}
function Ei(e, t, n) {
  return n === void 0
    ? 0
    : Math.abs(typeof e == "number" ? n(0) - n(e) : Mn(e, t));
}
function Ff(e, t, n) {
  return n === void 0 ? 0 : typeof e == "number" ? n(e) - n(0) : Mn(e, t);
}
function If(e, t, n, r) {
  return r === void 0
    ? 0
    : Math.abs(
        (typeof t == "number" ? r(t) : Mn(t, n)) -
          (typeof e == "number" ? r(e) : Mn(e, n))
      );
}
function gl(e) {
  const [t] = wr(),
    n = t.labels.find((r) => r.id === e);
  return n ? n.isVisible : !0;
}
function xl(e) {
  const { axisContext: t, plotWidth: n, plotHeight: r } = se(),
    { xAxis: i, yAxis: o, xShift: l, yShift: s } = e,
    [u, a] = tt(t, i, o, { onlyOrthogonal: !0 });
  return { xShift: Ff(l, n, u), yShift: Ff(s, r, a) };
}
function Y1({ color: e, id: t, width: n = 6 }) {
  return h.jsxs("defs", {
    children: [
      h.jsx("marker", {
        id: `marker-triangle-${t}`,
        viewBox: "0 0 10 10",
        refX: "5",
        refY: "5",
        markerWidth: n,
        markerHeight: n,
        orient: "auto-start-reverse",
        children: h.jsx("path", { fill: e, d: "M 0 0 L 10 5 L 0 10 z" }),
      }),
      h.jsx("marker", {
        id: `marker-circle-${t}`,
        viewBox: "0 0 10 10",
        refX: "5",
        refY: "5",
        markerWidth: n,
        markerHeight: n,
        children: h.jsx("circle", { cx: "5", cy: "5", r: "5", fill: e }),
      }),
      h.jsx("marker", {
        id: `marker-line-${t}`,
        viewBox: "0 0 10 10",
        refX: "5",
        refY: "5",
        markerWidth: n,
        markerHeight: n,
        orient: "auto",
        children: h.jsx("line", {
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
      xAxis: c = "x",
      yAxis: f = "y",
      ...d
    } = e,
    { x, y: g } = Cn({ x: t, y: n, xAxis: c, yAxis: f }),
    { x: v, y: E } = Cn({ x: r, y: i, xAxis: c, yAxis: f }),
    m = o !== "none" ? `url(#marker-${o}-${x}-${g}-${v}-${E})` : void 0,
    p = l !== "none" ? `url(#marker-${l}-${x}-${g}-${v}-${E})` : void 0;
  return h.jsxs("g", {
    children: [
      h.jsx(Y1, { color: s, id: `${x}-${g}-${v}-${E}`, width: a }),
      h.jsx("line", {
        x1: x,
        y1: g,
        x2: v,
        y2: E,
        stroke: s,
        strokeWidth: u,
        markerStart: m,
        markerEnd: p,
        ...d,
      }),
    ],
  });
}
function Q1(e) {
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
      onMouseLeave: c,
      xAxis: f = "x",
      yAxis: d = "y",
      ...x
    } = e,
    {
      min: g,
      max: v,
      q1: E,
      median: m,
      q3: p,
      width: y,
      y: w,
      horizontal: k,
    } = W1({ ...x, xAxis: f, yAxis: d }),
    S = w - y / 2,
    P = w + y / 2,
    C = Math.abs(p - E);
  return h.jsxs("g", {
    onMouseEnter: a,
    onMouseLeave: c,
    children: [
      h.jsx("line", {
        x1: k ? g : w,
        x2: k ? E : w,
        y1: k ? w : g,
        y2: k ? w : E,
        stroke: o,
        style: l,
      }),
      h.jsx("line", {
        x1: k ? p : w,
        x2: k ? v : w,
        y1: k ? w : p,
        y2: k ? w : v,
        stroke: o,
        style: l,
      }),
      h.jsx("rect", {
        x: k ? Math.min(E, p) : S,
        y: k ? S : Math.min(E, p),
        width: k ? C : y,
        height: k ? y : C,
        stroke: r,
        style: i,
        fill: "none",
      }),
      h.jsx("line", {
        x1: k ? m : S,
        x2: k ? m : P,
        y1: k ? S : m,
        y2: k ? P : m,
        stroke: t,
        style: n,
      }),
      h.jsx("rect", {
        x: k ? Math.min(E, p) : S,
        y: k ? S : Math.min(E, p),
        width: k ? C : y,
        height: k ? y : C,
        stroke: r,
        style: { ...i, fill: "none" },
        fill: "none",
      }),
      h.jsx("line", {
        x1: k ? g : S,
        x2: k ? g : P,
        y1: k ? S : g,
        y2: k ? P : g,
        stroke: s,
        style: u,
      }),
      h.jsx("line", {
        x1: k ? v : S,
        x2: k ? v : P,
        y1: k ? S : v,
        y2: k ? P : v,
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
      rx: c,
    } = Np({ cx: t, cy: n, rx: r, ry: r, xAxis: o, yAxis: l });
  return h.jsx("circle", { cx: u, cy: a, r: c, fill: i, ...s });
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
      ...c
    } = e,
    {
      cx: f,
      cy: d,
      rx: x,
      ry: g,
      rotation: v,
    } = V1({ x1: t, y1: n, y2: r, x2: i, width: l, xAxis: u, yAxis: a });
  return h.jsx("ellipse", {
    cx: f,
    cy: d,
    rx: x,
    ry: g,
    transform: `rotate(${v} 0 0)`,
    style: { ...s, transformOrigin: "center", transformBox: "fill-box" },
    fill: o,
    ...c,
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
      cy: c,
      rx: f,
      ry: d,
    } = Np({ cx: t, cy: n, rx: r, ry: i, xAxis: l, yAxis: s });
  return h.jsx("ellipse", { cx: a, cy: c, rx: f, ry: d, fill: o, ...u });
}
function q1(e) {
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
    { x: a, y: c } = Cn({ x: t, y: n, xAxis: l, yAxis: s });
  return h.jsx(Ni, {
    x: a,
    y: c,
    horizontalAlign: r,
    verticalAlign: i,
    style: o,
    children: u,
  });
}
function J1(e) {
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
    { x: a, y: c } = Cn({ x: t, y: r, xAxis: l, yAxis: s }),
    { x: f, y: d } = Cn({ x: n, y: i, xAxis: l, yAxis: s });
  return h.jsx("line", { x1: a, x2: f, y1: c, y2: d, stroke: o, ...u });
}
function b1(e) {
  const { points: t, color: n, xAxis: r = "x", yAxis: i = "y", ...o } = e,
    l = Lp({ points: t, xAxis: r, yAxis: i });
  return h.jsx("polygon", { fill: n, points: l, ...o });
}
function ev(e) {
  const { points: t, color: n, xAxis: r = "x", yAxis: i = "y", ...o } = e,
    l = Lp({ points: t, xAxis: r, yAxis: i });
  return h.jsx("polyline", { stroke: n, fill: "none", points: l, ...o });
}
function tv(e) {
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
      y: c,
      width: f,
      height: d,
    } = H1({ x1: t, y1: n, x2: r, y2: i, xAxis: l, yAxis: s });
  return h.jsx("rect", { x: a, y: c, width: f, height: d, fill: o, ...u });
}
const $p = {
    circle: Ap,
    square: Dp,
    diamond: zp,
    triangle: Op,
    cross: Rp,
    xmark: iv,
    pentagon: ov,
    star: lv,
    hexagon: sv,
  },
  nv = Math.PI / 2,
  rv = Math.PI * 2;
function Ap({ style: e, size: t }) {
  return h.jsx("circle", { r: t / 2, style: e });
}
function Dp({ style: e, size: t }) {
  const n = t / 2;
  return h.jsx("rect", { x: -n, y: -n, width: t, height: t, style: e });
}
function zp({ size: e, style: t }) {
  const n = e / 2;
  return h.jsx("polygon", {
    points: `0,${-n} ${n},0 0,${n} ${-n},0`,
    style: t,
  });
}
function Op({ style: e, size: t }) {
  const n = (Math.sqrt(3) * t) / 2,
    r = t / 2;
  return h.jsx("polygon", {
    transform: `translate(0, -${(t - n) / 2})`,
    points: `${-r},${r} ${r},${r} 0,${r - n}`,
    style: e,
  });
}
function Rp({ size: e, style: t }) {
  const n = e / 2;
  return h.jsxs("g", {
    strokeWidth: 1,
    style: t,
    children: [
      h.jsx("line", { x1: 0, x2: 0, y1: n, y2: -n }),
      h.jsx("line", { x1: -n, x2: n, y1: 0, y2: 0 }),
    ],
  });
}
function iv(e) {
  return h.jsx("g", { transform: "rotate(45)", children: h.jsx(Rp, { ...e }) });
}
function ov({ style: e, size: t }) {
  return h.jsx("polygon", { points: Array.from(qo(t, 5)).join(" "), style: e });
}
function lv({ style: e, size: t }) {
  const n = Array.from(qo(t, 5, 0)),
    r = Array.from(qo(t / 2.5, 5, (2 * Math.PI) / 10)),
    i = [];
  for (let o = 0; o < n.length; o++) i.push(n[o], r[o]);
  return h.jsx("polygon", { points: i.join(" "), style: e });
}
function sv({ style: e, size: t }) {
  return h.jsx("polygon", { points: Array.from(qo(t, 6)).join(" "), style: e });
}
function* qo(e, t, n = 0) {
  const r = e / 2;
  for (let i = 0; i < t; i++) {
    const o = (rv * i) / t - nv + n,
      l = r * Math.cos(o),
      s = r * Math.sin(o);
    yield `${l},${s}`;
  }
}
const uv = { triangle: av, circle: cv, diamond: fv, square: dv };
function av(e) {
  return h.jsx(Op, { size: e.size, style: e.style });
}
function cv(e) {
  return h.jsx(Ap, { size: e.size, style: e.style });
}
function fv(e) {
  return h.jsx(zp, { size: e.size, style: e.style });
}
function dv(e) {
  return h.jsx(Dp, { size: e.size, style: e.style });
}
function hv(e) {
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
      yAxis: c = "y",
    } = e,
    f = uv[t];
  if (!f) throw new Error(`Invalid shape: "${t}"`);
  const { x: d, y: x } = Cn({ x: n, y: r, xAxis: a, yAxis: c });
  return h.jsx("g", {
    onMouseEnter: i,
    onMouseLeave: o,
    transform: `translate(${d}, ${x})`,
    children: h.jsx(f, { size: s, style: { fill: l, ...u } }),
  });
}
function pv(e) {
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
  return h.jsx("text", { x: u, y: a, fill: i, ...s, children: r });
}
const Uf = {
  Arrow: G1,
  Circle: X1,
  DirectedEllipse: K1,
  Ellipse: Z1,
  Group: q1,
  Line: J1,
  Rectangle: tv,
  Shape: hv,
  Text: pv,
  Polyline: ev,
  Polygon: b1,
  BoxPlot: Q1,
};
function Fp(e) {
  return h.jsx(h.Fragment, { children: e.children });
}
function Ip(e, t, n, r) {
  return e || t ? 0 : mv(n, r);
}
function mv(e, t) {
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
const qn = _.createContext(null);
function yv(e) {
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
      const c = [];
      if (o)
        for (const { position: f } of r)
          c.push(
            h.jsx(
              "line",
              {
                x1: f,
                x2: f,
                y1: i === "top" ? t : -t,
                y2: "0",
                stroke: "black",
                strokeDasharray: "2,2",
                strokeOpacity: 0.5,
                style: n,
              },
              f
            )
          );
      if (l) {
        const d = (u == null ? void 0 : u.ticks(r.length * 5)) || [];
        for (const x of d) {
          const g = u == null ? void 0 : u(x);
          if (!g || r.map((v) => v.position).includes(g)) return null;
          c.push(
            h.jsx(
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
      return c;
    }, [i, t, o, r, u, l, s, n]);
  return h.jsx("g", { children: a });
}
function gv(e) {
  const { plotWidth: t, label: n, labelStyle: r, verticalAlign: i } = e;
  return h.jsx(Ni, {
    x: t / 2,
    y: 0,
    horizontalAlign: "middle",
    verticalAlign: i,
    children: h.jsx("text", { textAnchor: "middle", style: r, children: n }),
  });
}
function xv(e) {
  const { plotWidth: t, style: n } = e;
  return h.jsx("line", { x1: 0, x2: t, stroke: "black", style: n });
}
function Up(e) {
  const {
      primaryTicks: t,
      getPositions: n,
      secondaryTickLength: r,
      scale: i,
      secondaryTickStyle: o,
      style: l,
      ...s
    } = e,
    u = t.map((c) => {
      const { line: f, text: d } = n(c.position);
      return h.jsx(
        Hf,
        { line: f, style: l, text: d, ...s, children: c.label },
        c.position
      );
    });
  let a = [];
  if (r !== 0) {
    const d =
      (Math.abs(
        (i == null ? void 0 : i.range()[1]) -
          (i == null ? void 0 : i.range()[0])
      ) || 0) /
        t.length <
      50
        ? 5
        : 10;
    a =
      ((i == null ? void 0 : i.ticks(t.length * d)) || []).map((g) => {
        if (t.map((m) => m.position).includes(i(g))) return null;
        const { line: v, text: E } = n(i(g), !0);
        return h.jsx(
          Hf,
          { line: v, text: E, secondary: !0, style: o, ...s },
          String(g)
        );
      }) || [];
  }
  return h.jsxs(h.Fragment, { children: [a, u] });
}
function Hf(e) {
  const {
    line: { x1: t = 0, x2: n = 0, y1: r = 0, y2: i = 0 },
    text: { x1: o = 0, y1: l = 0 },
    children: s,
    strokeColor: u = "black",
    strokeHeight: a = 1,
    anchor: c = "end",
    secondary: f = !1,
    labelStyle: d,
    style: x,
    dominantBaseline: g = "middle",
  } = e;
  return h.jsxs("g", {
    children: [
      h.jsx("line", {
        x1: t,
        x2: f && t !== n ? n * a : n,
        y1: r,
        y2: f && r !== i ? i * a : i,
        stroke: u,
        strokeWidth: f ? 1 : 1.5,
        style: x,
      }),
      !f &&
        h.jsx("text", {
          x: o,
          y: l,
          textAnchor: c,
          dominantBaseline: g,
          style: d,
          children: s,
        }),
    ],
  });
}
function Na(e) {
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
      displayPrimaryGridLines: c,
      displaySecondaryGridLines: f,
      primaryGridLineStyle: d,
      secondaryGridLineStyle: x,
      label: g,
      labelStyle: v,
      axisRef: E,
      primaryTicks: m,
      position: p,
      tickLabelStyle: y,
      innerOffset: w,
      secondaryTickLength: k,
      scale: S,
      secondaryTickStyle: P,
    } = e,
    C = p === "bottom",
    D = 0,
    O = C ? `translate(0, ${a})` : void 0,
    N = ht();
  function H(L, z = !1) {
    const { y1: F, y2: X, textPosition: Te } = R(z);
    return {
      line: { x1: L, x2: L, y1: F, y2: X },
      text: { x1: L, y1: C ? Te : -Te },
    };
  }
  function R(L = !1) {
    const z = L ? k : n,
      F = C ? z : -z;
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
  const Y =
      c || f
        ? h.jsx(yv, {
            position: p,
            plotHeight: a,
            primaryTicks: m,
            style: d,
            primaryGrid: c,
            secondaryGrid: f,
            secondaryStyle: x,
            scale: S,
          })
        : null,
    K =
      !s && !i
        ? h.jsx(Up, {
            anchor: "middle",
            primaryTicks: m,
            getPositions: H,
            labelStyle: y,
            style: t,
            secondaryTickLength: k,
            scale: S,
            secondaryTickStyle: P,
            dominantBaseline: C ? "text-before-edge" : "text-after-edge",
          })
        : null,
    Q = !s && !l ? h.jsx(xv, { style: o, plotWidth: u }) : null,
    oe =
      !s && g
        ? h.jsx(gv, {
            plotWidth: u,
            label: g,
            labelStyle: v,
            verticalAlign: C ? "start" : "end",
          })
        : null,
    T = _.useContext(qn);
  return h.jsxs("g", {
    transform: O,
    children: [
      Y,
      h.jsxs("g", {
        ref: T,
        children: [
          h.jsx("g", { ref: N.ref, children: K }),
          h.jsx("g", { ref: E, children: Q }),
          h.jsx("g", {
            transform: `translate(0, ${(N.height - w) * (C ? 1 : -1)})`,
            children: oe,
          }),
        ],
      }),
    ],
  });
}
function vv(e) {
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
      const c = [];
      if (o)
        for (const { position: f } of r)
          c.push(
            h.jsx(
              "line",
              {
                x1: "0",
                x2: i === "left" ? t : -t,
                y1: f,
                y2: f,
                stroke: "black",
                strokeDasharray: "2,2",
                strokeOpacity: 0.5,
                style: n,
              },
              f
            )
          );
      if (l) {
        const d = (s == null ? void 0 : s.ticks(r.length * 5)) || [];
        for (const x of d) {
          const g = (s == null ? void 0 : s(x)) || 0;
          if (r.map((v) => v.position).includes(g)) return null;
          c.push(
            h.jsx(
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
      return c;
    }, [i, t, o, r, s, l, u, n]);
  return h.jsx("g", { children: a });
}
function wv(e) {
  const { transform: t = "", label: n, ...r } = e;
  return h.jsx("text", {
    ...r,
    transform: `${t}rotate(-90)`,
    textAnchor: "middle",
    children: n,
  });
}
function kv(e) {
  const { plotHeight: t, label: n, labelStyle: r, horizontalAlign: i } = e;
  return h.jsx(Ni, {
    x: 0,
    y: t / 2,
    horizontalAlign: i,
    verticalAlign: "middle",
    children: h.jsx(wv, { label: n, style: r }),
  });
}
function Sv(e) {
  const { plotHeight: t, style: n } = e;
  return h.jsx("line", { y1: 0, y2: t, stroke: "black", style: n });
}
function $a(e) {
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
      plotHeight: c,
      displayPrimaryGridLines: f,
      displaySecondaryGridLines: d,
      secondaryGridLineStyle: x,
      label: g,
      labelStyle: v,
      axisRef: E,
      primaryTicks: m,
      position: p,
      tickLabelStyle: y,
      innerOffset: w,
      secondaryTickLength: k,
      scale: S,
      secondaryTickStyle: P,
    } = e,
    C = p === "right",
    D = 3,
    O = C ? `translate(${a}, 0)` : void 0,
    N = ht();
  function H(L, z = !1) {
    const { x1: F, x2: X, textPosition: Te } = R(z);
    return {
      line: { y1: L, y2: L, x1: F, x2: X },
      text: { y1: L, x1: C ? Te : -Te },
    };
  }
  function R(L = !1) {
    const z = L ? k : n,
      F = C ? z : -z;
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
  const Y =
      f || d
        ? h.jsx(vv, {
            position: p,
            plotWidth: a,
            primaryTicks: m,
            style: o,
            primaryGrid: f,
            secondaryGrid: d,
            secondaryStyle: x,
            scale: S,
          })
        : null,
    K =
      !u && !i
        ? h.jsx(Up, {
            anchor: C ? "start" : "end",
            primaryTicks: m,
            getPositions: H,
            labelStyle: y,
            style: t,
            secondaryTickLength: k,
            scale: S,
            secondaryTickStyle: P,
          })
        : null,
    Q = !u && !s ? h.jsx(Sv, { style: l, plotHeight: c }) : null,
    oe =
      !u && g
        ? h.jsx(kv, {
            plotHeight: c,
            label: g,
            labelStyle: v,
            horizontalAlign: C ? "start" : "end",
          })
        : null,
    T = _.useContext(qn);
  return h.jsxs("g", {
    transform: O,
    children: [
      Y,
      h.jsxs("g", {
        ref: T,
        children: [
          h.jsx("g", { ref: N.ref, children: K }),
          h.jsx("g", { ref: E, children: Q }),
          h.jsx("g", {
            transform: `translate(${(N.width - w) * (C ? 1 : -1)}, 0)`,
            children: oe,
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
    s = v1(r, l, o, { tickFormat: n }),
    u = l === "vertical" ? $a : Na;
  return h.jsx(u, { scale: r, axisRef: o, primaryTicks: s, position: t, ...i });
}
const Hp = _.memo(Ev);
function Pv(e) {
  const { position: t, tickLabelFormat: n, scale: r, ...i } = e,
    o = _.useRef(null),
    l = t === "left" || t === "right" ? "vertical" : "horizontal",
    s = S1(r, l, o, { tickFormat: n }),
    u = l === "vertical" ? $a : Na;
  return h.jsx(u, { scale: r, axisRef: o, primaryTicks: s, position: t, ...i });
}
const Wp = _.memo(Pv);
function Cv(e) {
  const { position: t, tickLabelFormat: n, scale: r, ...i } = e,
    o = _.useRef(null),
    l = t === "left" || t === "right" ? "vertical" : "horizontal",
    s = E1(r, l, o, { tickFormat: n }),
    u = l === "vertical" ? $a : Na;
  return h.jsx(u, { scale: r, axisRef: o, primaryTicks: s, position: t, ...i });
}
const Vp = _.memo(Cv);
function gn({
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
  primaryGridLineStyle: c,
  displaySecondaryGridLines: f,
  secondaryGridLineStyle: d,
  labelStyle: x,
  hidden: g = !1,
  tickLabelStyle: v,
  tickLabelFormat: E = s === "time" ? void 0 : String,
  hiddenLine: m = !1,
  lineStyle: p,
  hiddenTicks: y = !1,
  tickPosition: w = "outer",
  primaryTickLength: k = 5,
  primaryTickStyle: S,
  secondaryTickLength: P = 3,
  secondaryTickStyle: C,
}) {
  const D = $i(),
    { axisContext: O, plotWidth: N, plotHeight: H } = se(),
    R = ["top", "bottom"].includes(t) ? "x" : "y",
    Y = e || R,
    K = Ip(g, y, w, k);
  _.useEffect(
    () => (
      D({
        type: "addAxis",
        payload: {
          id: Y,
          position: t,
          min: n,
          max: r,
          paddingStart: i,
          paddingEnd: o,
          flip: l,
          scale: s,
          innerOffset: K,
          tickLabelFormat: E,
        },
      }),
      () => D({ type: "removeAxis", payload: { id: Y } })
    ),
    [D, l, K, r, n, o, i, t, s, Y, E]
  );
  const Q = O[Y];
  if (!Q) return null;
  const oe = {
    hidden: g,
    plotWidth: N,
    plotHeight: H,
    displayPrimaryGridLines: a,
    label: u,
    labelStyle: x,
    tickLabelStyle: v,
    position: t,
    hiddenLine: m,
    primaryGridLineStyle: c,
    displaySecondaryGridLines: f,
    secondaryGridLineStyle: d,
    lineStyle: p,
    hiddenTicks: y,
    tickPosition: w,
    primaryTickLength: k,
    primaryTickStyle: S,
    innerOffset: K,
    secondaryTickLength: P,
    secondaryTickStyle: C,
  };
  return s === "linear"
    ? h.jsx(Hp, { ...oe, tickLabelFormat: E, scale: Q.scale })
    : s === "time"
    ? h.jsx(Vp, { ...oe, tickLabelFormat: E, scale: Q.scale })
    : h.jsx(Wp, { ...oe, tickLabelFormat: E, scale: Q.scale });
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
function jv(e) {
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
    { axisContext: c, plotWidth: f, plotHeight: d } = se(),
    x = Ip(n, s, o, r),
    g = c[t];
  if (!g) return null;
  const v = Mv(g.position),
    { type: E, scale: m, tickLabelFormat: p } = g,
    y = u ?? p,
    w = {
      plotWidth: f,
      plotHeight: d,
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
  return E === "linear"
    ? h.jsx(Hp, { ...w, tickLabelFormat: y, scale: m })
    : E === "time"
    ? h.jsx(Vp, { ...w, tickLabelFormat: y, scale: m })
    : h.jsx(Wp, { ...w, tickLabelFormat: y, scale: m });
}
function vl({
  title: e,
  titleStyle: t,
  titleClass: n,
  subtitle: r,
  subtitleStyle: i,
  subtitleClass: o,
  position: l = "top",
}) {
  const { width: s, height: u } = se(),
    a = $i(),
    c = {
      dominantBaseline: "hanging",
      textAnchor: "middle",
      fontSize: "16px",
      fontWeight: "bold",
    },
    f = {
      dominantBaseline: "hanging",
      textAnchor: "middle",
      fontSize: "14px",
      color: "gray",
    },
    d = ht();
  return (
    _.useEffect(
      () => (
        a({ type: "addHeading", payload: { position: l } }),
        () => a({ type: "removeHeading" })
      ),
      [a, l]
    ),
    h.jsxs(Ni, {
      x: s / 2,
      y: l === "top" ? 0 : u,
      horizontalAlign: "middle",
      verticalAlign: l === "top" ? "start" : "end",
      children: [
        h.jsx("text", {
          ref: d.ref,
          style: { ...c, ...t },
          className: n,
          children: e,
        }),
        r &&
          h.jsx("text", {
            transform: `translate(0, ${d.height})`,
            style: { ...f, ...i },
            className: o,
            children: r,
          }),
      ],
    })
  );
}
vl.defaultProps = { position: "top" };
const Bp = _.createContext(0);
function zn(e, t, n, r) {
  if (e[t] !== void 0) {
    if (e[n] !== void 0)
      throw new Error(
        `${t} and ${n} should't be both defined for the position ${r}`
      );
    return { key: t, value: e[t] };
  }
  return e[n] !== void 0 ? { key: n, value: e[n] } : {};
}
function Tv(e, t, n, r, i) {
  switch (e) {
    case "embedded": {
      const { key: o = "top", value: l = 10 } = zn(t, "top", "bottom", e),
        { key: s = "left", value: u = 10 } = zn(t, "left", "right", e),
        a = s === "right" ? n - u : u,
        c = o === "bottom" ? r - l : l;
      return {
        x: a,
        y: c,
        horizontalAlign: s === "left" ? "start" : "end",
        verticalAlign: o === "top" ? "start" : "end",
      };
    }
    case "top": {
      const { value: o = n / 2 } = zn(t, "left", "right", e),
        l = o,
        s = -(t.bottom || 0) - i;
      return { x: l, y: s, horizontalAlign: "middle", verticalAlign: "end" };
    }
    case "right": {
      const { key: o = "top", value: l = r / 2 } = zn(t, "top", "bottom", e),
        s = o === "bottom" ? -l : l;
      return {
        x: n + (t.left || 0) + i,
        y: s,
        horizontalAlign: "start",
        verticalAlign: "middle",
      };
    }
    case "bottom": {
      const { value: o = n / 2 } = zn(t, "left", "right", e),
        l = o,
        s = r + (t.top || 0) + i;
      return { x: l, y: s, horizontalAlign: "middle", verticalAlign: "start" };
    }
    case "left": {
      const { key: o = "top", value: l = r / 2 } = zn(t, "top", "bottom", e),
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
function Yp(e) {
  const {
      position: t = "embedded",
      margin: n = 10,
      onClick: r,
      lineStyle: i = {},
      showHide: o = !1,
      labelStyle: l = {},
      ...s
    } = e,
    { plotWidth: u, plotHeight: a } = se(),
    c = $i(),
    [f, d] = wr(),
    x = _.useContext(Bp),
    g = _.useMemo(() => Tv(t, s, u, a, x), [t, s, u, a, x]);
  _.useEffect(
    () => (
      c({ type: "addLegend", payload: { position: t, margin: n } }),
      () => c({ type: "removeLegend" })
    ),
    [c, t, n]
  );
  function v(E, m) {
    r == null || r({ event: E, id: m }),
      o && d({ type: "TOGGLE_VISIBILITY", payload: { id: m } });
  }
  return h.jsx(Ni, {
    ...g,
    children: f.labels.map((E, m) => {
      const p = yn({ fontSize: "16px" }, l, { id: E.id }),
        y = yn({}, i, { id: E.id }),
        w = Number.parseInt(String(p.fontSize), 10),
        k = 10,
        S = m * w + w / 2 + 3;
      if (E.range)
        return h.jsxs(
          "g",
          {
            onClick: (C) => v(C, E.id),
            transform: `translate(${k}, 0)`,
            style: { opacity: E.isVisible ? "1" : "0.6" },
            children: [
              Lv({
                index: m,
                rangeColor: E.range.rangeColor,
                lineColor: E.colorLine,
                style: y,
                height: w,
              }),
              h.jsx("text", {
                style: p,
                x: 30,
                y: `${(m + 1) * w}`,
                children: E.label,
              }),
            ],
          },
          m
        );
      let P;
      return (
        E.shape && (P = $p[E.shape.figure]),
        h.jsxs(
          "g",
          {
            onClick: (C) => v(C, E.id),
            transform: `translate(${k}, 0)`,
            style: { opacity: E.isVisible ? "1" : "0.6" },
            children: [
              _v({ index: m, color: E.colorLine, style: y, height: w }),
              h.jsx("g", {
                transform: `translate(${k - 1}, ${S})`,
                children:
                  E.shape &&
                  P &&
                  !E.shape.hidden &&
                  h.jsx(P, { size: 10, style: { fill: E.shape.color } }),
              }),
              h.jsx("text", {
                style: p,
                x: 30,
                y: `${(m + 1) * w}`,
                children: E.label,
              }),
            ],
          },
          m
        )
      );
    }),
  });
}
function _v(e) {
  const { index: t, color: n, style: r = {}, height: i = 16 } = e,
    o = 0,
    { strokeWidth: l = "2px" } = r,
    s = t * i + i / 2 + 3;
  return h.jsx("line", {
    x1: o,
    x2: o + 20,
    y1: s,
    y2: s,
    stroke: n,
    style: { ...r, strokeWidth: l },
  });
}
function Lv(e) {
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
  return h.jsxs("g", {
    transform: `translate(${u}, ${a})`,
    children: [
      h.jsx("rect", { width: "20", height: s * 3, fill: n }),
      h.jsx("line", {
        style: i,
        x1: 0,
        y: 0,
        x2: 20,
        y2: 0,
        stroke: r,
        strokeWidth: s,
      }),
      h.jsx("line", {
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
function Nv(e) {
  const {
      xAxis: t = "x",
      yAxis: n = "y",
      data: r,
      hidden: i,
      transform: o,
      ...l
    } = e,
    { axisContext: s } = se(),
    [u, a] = tt(s, t, n),
    c = _.useMemo(
      () =>
        i || u === void 0 || a === void 0
          ? null
          : r.map((d, x) => {
              const g = Df(d.xError),
                v = Df(d.yError);
              return h.jsx(
                $v,
                {
                  origin: { x: u(d.x), y: a(d.y) },
                  bottom: v ? a(d.y - (v[0] || 0)) : null,
                  top: v ? a(d.y + (v[1] || 0)) : null,
                  left: g ? u(d.x - (g[0] || 0)) : null,
                  right: g ? u(d.x + (g[1] || 0)) : null,
                  ...l,
                },
                `ErrorBars-${x}`
              );
            }),
      [r, u, a, i, l]
    );
  return h.jsx("g", { transform: o, children: c });
}
function $v(e) {
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
  return h.jsxs("g", {
    children: [
      n != null &&
        h.jsxs("g", {
          children: [
            h.jsx("line", {
              x1: t.x,
              x2: t.x,
              y1: t.y,
              y2: n,
              stroke: a,
              ...l,
            }),
            h.jsx("line", {
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
        h.jsxs("g", {
          children: [
            h.jsx("line", {
              x1: t.x,
              x2: t.x,
              y1: t.y,
              y2: r,
              stroke: a,
              ...l,
            }),
            h.jsx("line", {
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
        h.jsxs("g", {
          children: [
            h.jsx("line", {
              x1: t.x,
              x2: i,
              y1: t.y,
              y2: t.y,
              stroke: a,
              ...l,
            }),
            h.jsx("line", {
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
        h.jsxs("g", {
          children: [
            h.jsx("line", {
              x1: t.x,
              x2: o,
              y1: t.y,
              y2: t.y,
              stroke: a,
              ...l,
            }),
            h.jsx("line", {
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
function Aa(e) {
  const t = $i(),
    { colorScaler: n } = se(),
    [, r] = wr(),
    i = kr(e.id, "series"),
    {
      xAxis: o = "x",
      yAxis: l = "y",
      data: s,
      label: u,
      hidden: a,
      displayErrorBars: c = !1,
      xShift: f = 0,
      yShift: d = 0,
      ...x
    } = e,
    {
      markerShape: g = "circle",
      markerStyle: v = {},
      errorBarsStyle: E,
      errorBarsCapStyle: m,
      errorBarsCapSize: p,
    } = x,
    y = gl(i),
    { xShift: w, yShift: k } = xl({ xAxis: o, yAxis: l, xShift: f, yShift: d }),
    S = `translate(${w},${k})`;
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
      const [D, O] = br(s, (K) => K.x),
        [N, H] = br(s, (K) => K.y);
      return (
        t({
          type: "addSeries",
          payload: {
            id: i,
            x: { min: D, max: O, axisId: o, shift: f },
            y: { min: N, max: H, axisId: l, shift: d },
            label: u,
            data: s,
          },
        }),
        () => t({ type: "removeSeries", payload: { id: i } })
      );
    }, [t, i, s, o, l, u, f, d]),
    a)
  )
    return null;
  const P = { data: s, xAxis: o, yAxis: l },
    C = { hidden: !c, style: E, capStyle: m, capSize: p, transform: S };
  return y
    ? h.jsxs("g", {
        children: [
          h.jsx(Nv, { ...P, ...C }),
          h.jsx(Av, { ...x, ...P, id: i, transform: S }),
        ],
      })
    : null;
}
function Av({
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
  lineStyle: c = {},
  displayLines: f = !1,
  transform: d,
}) {
  const { axisContext: x, colorScaler: g } = se(),
    [v, E] = tt(x, n, r),
    m = _.useMemo(() => {
      if (v === void 0 || E === void 0) return null;
      const p = g(e),
        y = { fill: p, stroke: p };
      return t.map((k, S) => {
        const P = yn(y, l, k, S, t),
          C = $p[L1(i, k, S, t)],
          D = N1(s, k, S, t),
          O = yn({}, u, k, S, t),
          N = [];
        if (f) {
          const H = yn({}, c, k, S, t),
            R = S > 0 ? zf(k, t[S - 1]) : void 0,
            Y = t[S + 1] ? zf(k, t[S + 1]) : void 0,
            K = R
              ? h.jsx(
                  "line",
                  {
                    x1: 0,
                    y1: 0,
                    x2: v(R.x) - v(k.x),
                    y2: E(R.y) - E(k.y),
                    style: { stroke: P.fill, ...H },
                  },
                  `markers-${S}-previous`
                )
              : null,
            Q = Y
              ? h.jsx(
                  "line",
                  {
                    x1: 0,
                    y1: 0,
                    x2: v(Y.x) - v(k.x),
                    y2: E(Y.y) - E(k.y),
                    style: { stroke: P.fill, ...H },
                  },
                  `markers-${S}-next`
                )
              : null;
          N.push(K, Q);
        }
        return h.jsxs(
          "g",
          {
            transform: `translate(${v(k.x)}, ${E(k.y)})`,
            children: [
              N,
              a ? h.jsx(C, { size: o, style: { stroke: P.fill, ...P } }) : null,
              D ? h.jsx("text", { style: O, children: D }) : null,
            ],
          },
          `markers-${S}`
        );
      });
    }, [v, E, g, e, t, l, i, s, u, c, f, a, o]);
  return m
    ? h.jsx("g", { transform: d, className: "markers", children: m })
    : null;
}
function Dv(e) {
  var t;
  const [, n] = wr(),
    { colorScaler: r } = se(),
    i = kr(e.id, "series"),
    { lineStyle: o = {}, displayMarkers: l = !1, hidden: s, ...u } = e,
    {
      xAxis: a = "x",
      yAxis: c = "y",
      xShift: f = "0",
      yShift: d = "0",
      pointLabel: x,
      displayErrorBars: g,
      label: v,
      markerStyle: E,
      markerShape: m,
    } = u,
    { xShift: p, yShift: y } = xl({ xAxis: a, yAxis: c, xShift: f, yShift: d }),
    w = {
      id: i,
      data: e.data,
      xAxis: a,
      yAxis: c,
      lineStyle: yn({}, o, { id: i }),
      transform: `translate(${p},${y})`,
    },
    k =
      o != null && o.stroke ? (o == null ? void 0 : o.stroke.toString()) : r(i),
    S =
      ((t = E == null ? void 0 : E.fill) === null || t === void 0
        ? void 0
        : t.toString()) || r(i),
    P = m || "circle",
    C = _.useMemo(() => ({ color: S, figure: P, hidden: !l }), [S, l, P]),
    D = gl(i);
  return (
    _.useEffect(() => {
      if (!s)
        return (
          n({
            type: "ADD_LEGEND_LABEL",
            payload: { id: i, label: v, colorLine: k, shape: C },
          }),
          () => n({ type: "REMOVE_LEGEND_LABEL", payload: { id: i } })
        );
    }, [k, n, v, C, i, s]),
    s
      ? null
      : h.jsxs("g", {
          children: [
            D && h.jsx(zv, { ...w }),
            h.jsx(Aa, {
              ...u,
              hidden: !l && !x && !g,
              displayMarkers: l,
              id: i,
            }),
          ],
        })
  );
}
function zv({
  id: e,
  data: t,
  xAxis: n,
  yAxis: r,
  lineStyle: i,
  transform: o,
}) {
  const { axisContext: l, colorScaler: s } = se(),
    [u, a] = tt(l, n, r),
    c = s(e);
  if (u === void 0 || a === void 0) return null;
  const f = { stroke: c, strokeWidth: 2, ...i };
  return h.jsx("g", {
    transform: o,
    children: t.map(({ x: d, y: x }) =>
      h.jsx(
        "line",
        { style: f, x1: u(d), x2: u(d), y1: a(x), y2: a(0) },
        `${d}-${x}`
      )
    ),
  });
}
function de(e) {
  return function () {
    return e;
  };
}
const pu = Math.PI,
  mu = 2 * pu,
  sn = 1e-6,
  Ov = mu - sn;
function Gp(e) {
  this._ += e[0];
  for (let t = 1, n = e.length; t < n; ++t) this._ += arguments[t] + e[t];
}
function Rv(e) {
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
class Fv {
  constructor(t) {
    (this._x0 = this._y0 = this._x1 = this._y1 = null),
      (this._ = ""),
      (this._append = t == null ? Gp : Rv(t));
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
      c = l - t,
      f = s - n,
      d = c * c + f * f;
    if (this._x1 === null) this._append`M${(this._x1 = t)},${(this._y1 = n)}`;
    else if (d > sn)
      if (!(Math.abs(f * u - a * c) > sn) || !o)
        this._append`L${(this._x1 = t)},${(this._y1 = n)}`;
      else {
        let x = r - l,
          g = i - s,
          v = u * u + a * a,
          E = x * x + g * g,
          m = Math.sqrt(v),
          p = Math.sqrt(d),
          y = o * Math.tan((pu - Math.acos((v + d - E) / (2 * m * p))) / 2),
          w = y / p,
          k = y / m;
        Math.abs(w - 1) > sn && this._append`L${t + w * c},${n + w * f}`,
          this._append`A${o},${o},0,0,${+(f * x > c * g)},${(this._x1 =
            t + k * u)},${(this._y1 = n + k * a)}`;
      }
  }
  arc(t, n, r, i, o, l) {
    if (((t = +t), (n = +n), (r = +r), (l = !!l), r < 0))
      throw new Error(`negative radius: ${r}`);
    let s = r * Math.cos(i),
      u = r * Math.sin(i),
      a = t + s,
      c = n + u,
      f = 1 ^ l,
      d = l ? i - o : o - i;
    this._x1 === null
      ? this._append`M${a},${c}`
      : (Math.abs(this._x1 - a) > sn || Math.abs(this._y1 - c) > sn) &&
        this._append`L${a},${c}`,
      r &&
        (d < 0 && (d = (d % mu) + mu),
        d > Ov
          ? this._append`A${r},${r},0,1,${f},${t - s},${
              n - u
            }A${r},${r},0,1,${f},${(this._x1 = a)},${(this._y1 = c)}`
          : d > sn &&
            this._append`A${r},${r},0,${+(d >= pu)},${f},${(this._x1 =
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
function Qp(e) {
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
    () => new Fv(t)
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
function qp(e) {
  return e[0];
}
function Jp(e) {
  return e[1];
}
function bp(e, t) {
  var n = de(!0),
    r = null,
    i = Zp,
    o = null,
    l = Qp(s);
  (e = typeof e == "function" ? e : e === void 0 ? qp : de(e)),
    (t = typeof t == "function" ? t : t === void 0 ? Jp : de(t));
  function s(u) {
    var a,
      c = (u = Xp(u)).length,
      f,
      d = !1,
      x;
    for (r == null && (o = i((x = l()))), a = 0; a <= c; ++a)
      !(a < c && n((f = u[a]), a, u)) === d &&
        ((d = !d) ? o.lineStart() : o.lineEnd()),
        d && o.point(+e(f, a, u), +t(f, a, u));
    if (x) return (o = null), x + "" || null;
  }
  return (
    (s.x = function (u) {
      return arguments.length
        ? ((e = typeof u == "function" ? u : de(+u)), s)
        : e;
    }),
    (s.y = function (u) {
      return arguments.length
        ? ((t = typeof u == "function" ? u : de(+u)), s)
        : t;
    }),
    (s.defined = function (u) {
      return arguments.length
        ? ((n = typeof u == "function" ? u : de(!!u)), s)
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
function Iv(e, t, n) {
  var r = null,
    i = de(!0),
    o = null,
    l = Zp,
    s = null,
    u = Qp(a);
  (e = typeof e == "function" ? e : e === void 0 ? qp : de(+e)),
    (t = typeof t == "function" ? t : de(t === void 0 ? 0 : +t)),
    (n = typeof n == "function" ? n : n === void 0 ? Jp : de(+n));
  function a(f) {
    var d,
      x,
      g,
      v = (f = Xp(f)).length,
      E,
      m = !1,
      p,
      y = new Array(v),
      w = new Array(v);
    for (o == null && (s = l((p = u()))), d = 0; d <= v; ++d) {
      if (!(d < v && i((E = f[d]), d, f)) === m)
        if ((m = !m)) (x = d), s.areaStart(), s.lineStart();
        else {
          for (s.lineEnd(), s.lineStart(), g = d - 1; g >= x; --g)
            s.point(y[g], w[g]);
          s.lineEnd(), s.areaEnd();
        }
      m &&
        ((y[d] = +e(E, d, f)),
        (w[d] = +t(E, d, f)),
        s.point(r ? +r(E, d, f) : y[d], n ? +n(E, d, f) : w[d]));
    }
    if (p) return (s = null), p + "" || null;
  }
  function c() {
    return bp().defined(i).curve(l).context(o);
  }
  return (
    (a.x = function (f) {
      return arguments.length
        ? ((e = typeof f == "function" ? f : de(+f)), (r = null), a)
        : e;
    }),
    (a.x0 = function (f) {
      return arguments.length
        ? ((e = typeof f == "function" ? f : de(+f)), a)
        : e;
    }),
    (a.x1 = function (f) {
      return arguments.length
        ? ((r = f == null ? null : typeof f == "function" ? f : de(+f)), a)
        : r;
    }),
    (a.y = function (f) {
      return arguments.length
        ? ((t = typeof f == "function" ? f : de(+f)), (n = null), a)
        : t;
    }),
    (a.y0 = function (f) {
      return arguments.length
        ? ((t = typeof f == "function" ? f : de(+f)), a)
        : t;
    }),
    (a.y1 = function (f) {
      return arguments.length
        ? ((n = f == null ? null : typeof f == "function" ? f : de(+f)), a)
        : n;
    }),
    (a.lineX0 = a.lineY0 =
      function () {
        return c().x(e).y(t);
      }),
    (a.lineY1 = function () {
      return c().x(e).y(n);
    }),
    (a.lineX1 = function () {
      return c().x(r).y(t);
    }),
    (a.defined = function (f) {
      return arguments.length
        ? ((i = typeof f == "function" ? f : de(!!f)), a)
        : i;
    }),
    (a.curve = function (f) {
      return arguments.length ? ((l = f), o != null && (s = l(o)), a) : l;
    }),
    (a.context = function (f) {
      return arguments.length
        ? (f == null ? (o = s = null) : (s = l((o = f))), a)
        : o;
    }),
    a
  );
}
function Uv(e) {
  const [, t] = wr(),
    { colorScaler: n } = se(),
    r = kr(e.id, "series"),
    { lineStyle: i = {}, displayMarkers: o = !1, hidden: l, ...s } = e,
    {
      xAxis: u = "x",
      yAxis: a = "y",
      xShift: c = "0",
      yShift: f = "0",
      pointLabel: d,
      displayErrorBars: x,
      label: g,
      markerStyle: v,
      markerShape: E,
    } = s,
    { xShift: m, yShift: p } = xl({ xAxis: u, yAxis: a, xShift: c, yShift: f }),
    y = yn({}, i, { id: r }),
    w = gl(r);
  if (
    (_.useEffect(() => {
      var S, P;
      if (!l)
        return (
          t({
            type: "ADD_LEGEND_LABEL",
            payload: {
              id: r,
              label: g,
              colorLine:
                ((S = y == null ? void 0 : y.stroke) === null || S === void 0
                  ? void 0
                  : S.toString()) || n(r),
              shape: {
                color:
                  ((P = v == null ? void 0 : v.fill) === null || P === void 0
                    ? void 0
                    : P.toString()) || n(r),
                figure: E || "circle",
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
      E,
      v == null ? void 0 : v.fill,
    ]),
    l)
  )
    return null;
  const k = {
    id: r,
    data: e.data,
    xAxis: u,
    yAxis: a,
    lineStyle: y,
    transform: `translate(${m},${p})`,
  };
  return h.jsxs("g", {
    children: [
      w && h.jsx(Hv, { ...k }),
      h.jsx(Aa, { ...s, hidden: !o && !d && !x, displayMarkers: o, id: r }),
    ],
  });
}
const wl = _.memo(Uv);
function Hv({
  id: e,
  data: t,
  xAxis: n,
  yAxis: r,
  lineStyle: i,
  transform: o,
}) {
  const { axisContext: l, colorScaler: s } = se(),
    [u, a] = tt(l, n, r),
    c = s(e),
    f = _.useMemo(
      () =>
        u === void 0 || a === void 0
          ? null
          : bp()
              .x((g) => u(g.x))
              .y((g) => a(g.y))(t),
      [t, u, a]
    );
  if (!f) return null;
  const d = { stroke: c, strokeWidth: 2, ...i };
  return h.jsx("path", { transform: o, style: d, d: f, fill: "none" });
}
function Wv(e) {
  const { getY: t, xAxis: n = "x", id: r, ...i } = e,
    o = `~${kr(r, "series")}`,
    { axisContext: l, plotWidth: s, plotHeight: u } = se(),
    a = l[n],
    c = 1,
    f = _.useMemo(() => {
      const d = [];
      if (a) {
        const g = (a ? a.position === "top" || a.position === "bottom" : !1)
            ? s
            : u,
          v = a.scale;
        for (let E = 0; E <= g; E += c) {
          const m = Hr(v.invert(E));
          d.push({ x: m, y: t(m) });
        }
        return d;
      }
      return [{ x: 0, y: 0 }];
    }, [a == null ? void 0 : a.domain[0], a == null ? void 0 : a.domain[1]]);
  return h.jsx(wl, { data: f, id: o, ...i });
}
function Vv(e) {
  const t = kr(e.id, "series"),
    [, n] = wr(),
    { colorScaler: r } = se(),
    {
      lineStyle: i = { fill: r(t), fillOpacity: 0.5 },
      hidden: o,
      xAxis: l = "x",
      yAxis: s = "y",
      data: u,
      label: a,
      xShift: c = "0",
      yShift: f = "0",
    } = e,
    { xShift: d, yShift: x } = xl({ xAxis: l, yAxis: s, xShift: c, yShift: f }),
    g = $i();
  _.useEffect(() => {
    const [m, p] = br(u, (D) => D.x),
      [y, w] = br(u, (D) => D.y1),
      [k, S] = br(u, (D) => D.y2),
      P = { min: m, max: p, shift: c, axisId: l },
      C = { min: Math.min(y, k), max: Math.max(w, S), shift: f, axisId: s };
    return (
      g({ type: "addSeries", payload: { id: t, x: P, y: C, label: a } }),
      () => g({ type: "removeSeries", payload: { id: t } })
    );
  }, [g, t, u, l, s, a, c, f]);
  const v = gl(t);
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
  const E = {
    id: t,
    data: u,
    xAxis: l,
    yAxis: s,
    lineStyle: i,
    transform: `translate(${d},${x})`,
  };
  return v ? h.jsx(Yv, { ...E }) : null;
}
const Bv = _.memo(Vv);
function Yv({ data: e, xAxis: t, yAxis: n, lineStyle: r, transform: i }) {
  const { axisContext: o } = se(),
    [l, s] = tt(o, t, n),
    u = _.useMemo(
      () =>
        l === void 0 || s === void 0
          ? null
          : Iv()
              .x((f) => l(f.x))
              .y0((f) => s(f.y1))
              .y1((f) => s(f.y2))(e),
      [e, l, s]
    );
  if (!u) return null;
  const a = { strokeWidth: 2, ...r };
  return h.jsx("path", { transform: i, style: a, d: u, fill: "none" });
}
function Gv(e) {
  return h.jsx(h.Fragment, { children: e.children });
}
function Qv(e) {
  let t = null,
    n = null,
    r = null,
    i = null,
    o = [],
    l = null,
    s = null,
    u = [],
    a = [];
  for (let c of _.Children.toArray(e)) {
    if (typeof c != "object" || !_.isValidElement(c))
      throw (
        (console.error("Invalid Plot child:", c),
        new Error("invalid Plot child"))
      );
    if (
      c.type === Gv ||
      c.type === Wv ||
      c.type === wl ||
      c.type === Aa ||
      c.type === Bv ||
      c.type === Dv
    )
      u.push(c);
    else if (c.type === Fp) a.push(c);
    else if (c.type === gn)
      switch (c.props.position) {
        case "top": {
          if (t !== null) throw new Error("Plot can only have one top axis");
          t = c;
          break;
        }
        case "right": {
          if (n !== null) throw new Error("Plot can only have one right axis");
          n = c;
          break;
        }
        case "bottom": {
          if (r !== null) throw new Error("Plot can only have one bottom axis");
          r = c;
          break;
        }
        case "left": {
          if (i !== null) throw new Error("Plot can only have one left axis");
          i = c;
          break;
        }
        default:
          throw new Error("unreachable");
      }
    else if (c.type === jv) {
      if (o.length === 2)
        throw new Error("Plot can have at most two parallel axes");
      o.push(c);
    } else if (c.type === vl) {
      if (l !== null) throw new Error("Plot can only have one Heading element");
      l = c;
    } else if (c.type === Yp) {
      if (s !== null) throw new Error("Plot can only have one Legend element");
      s = c;
    } else
      throw (
        (console.error("Invalid Plot child:", c),
        new Error("invalid plot child"))
      );
  }
  !r && !t && (r = h.jsx(gn, { position: "bottom" })),
    !i && !n && (i = h.jsx(gn, { position: "left" }));
  for (const c of o) {
    const f = c.props.id;
    if ((t == null ? void 0 : t.props.id) === f) {
      if (r !== null) throw new Error("Plot can only have one bottom axis");
      r = c;
    }
    if ((n == null ? void 0 : n.props.id) === f) {
      if (i !== null) throw new Error("Plot can only have one left axis");
      i = c;
    }
    if ((r == null ? void 0 : r.props.id) === f) {
      if (t !== null) throw new Error("Plot can only have one top axis");
      t = c;
    }
    if ((i == null ? void 0 : i.props.id) === f) {
      if (n !== null) throw new Error("Plot can only have one right axis");
      n = c;
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
  legendPosition: c,
  legendMargin: f,
  legendWidth: d,
  legendHeight: x,
}) {
  const { top: g = 0, right: v = 0, bottom: E = 0, left: m = 0 } = n;
  return _.useMemo(() => {
    const p = Kv(r),
      y = i - p.top,
      w = o - p.right,
      k = l - p.bottom,
      S = s - p.left;
    let P = e - m - S - v - w,
      C = t - g - a - y - E - k,
      D = m + S,
      O = g + y;
    u === "top" && (O += a);
    const N = x + f,
      H = d + f;
    let R = 0;
    switch (c) {
      case "top":
        (C -= N), (O += N), (R = y + f);
        break;
      case "right":
        (P -= H), (R = w + f);
        break;
      case "bottom":
        (C -= N), (R = k + f);
        break;
      case "left":
        (P -= H), (D += H), (R = S + f);
        break;
    }
    return {
      plotWidth: P,
      plotHeight: C,
      leftOffset: D,
      topOffset: O,
      legendOffset: R,
    };
  }, [e, t, g, v, E, m, i, o, l, s, u, a, c, f, d, x, r]);
}
function Kv(e) {
  const t = { top: 0, right: 0, bottom: 0, left: 0 };
  for (const n of Object.values(e)) t[n.position] = n.innerOffset;
  return t;
}
const Zv = new Set(["bottom", "top"]);
function qv(e, t, n, r) {
  const { clientX: i, clientY: o, movementX: l, movementY: s } = e,
    { left: u, top: a } = r.getBoundingClientRect(),
    c = i - u,
    f = o - a,
    d = {},
    x = {},
    g = {},
    v = {};
  for (const E in t) {
    const { scale: m, clampInDomain: p, position: y, domain: w } = t[E];
    Zv.has(y)
      ? ((d[E] = Hr(m.invert(c))), (v[E] = Hr(m.invert(l)) - w[0]))
      : ((d[E] = Hr(m.invert(f))), (v[E] = Hr(m.invert(s)) - w[1])),
      (x[E] = p(d[E])),
      (g[E] = w);
  }
  return {
    event: e,
    coordinates: d,
    clampedCoordinates: x,
    movement: v,
    getClosest(E) {
      return Jv(E, { x: c, y: f }, n, t);
    },
    domains: g,
  };
}
function Jv(e, t, n, r) {
  let i = {};
  switch (e) {
    case "x": {
      for (const { id: o, x: l, data: s, label: u } of n)
        if (s) {
          const a = is(s, t, (c, f) => {
            const { scale: d } = r[l.axisId],
              x = f[l.axisId];
            return Math.abs(d(c.x) - x);
          });
          i[o] = { point: a, label: u, axis: r[l.axisId] };
        }
      break;
    }
    case "y": {
      for (const { id: o, y: l, data: s, label: u } of n)
        if (s) {
          const a = is(s, t, (c, f) => {
            const { scale: d } = r[l.axisId],
              x = f[l.axisId];
            return Math.abs(d(c.y) - x);
          });
          i[o] = { point: a, label: u, axis: r[l.axisId] };
        }
      break;
    }
    case "euclidean": {
      for (const { id: o, x: l, y: s, data: u, label: a } of n)
        if (u) {
          const c = is(u, t, (f, d) => {
            const { scale: x } = r[l.axisId],
              { scale: g } = r[s.axisId],
              v = d[l.axisId],
              E = d[s.axisId];
            return Wr([x(f.x), g(f.y)], [v, E]);
          });
          i[o] = {
            point: c,
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
const bv = {
    pointerenter: "onPointerEnter",
    pointerdown: "onPointerDown",
    pointermove: "onPointerMove",
    pointerup: "onPointerUp",
    pointerleave: "onPointerLeave",
    click: "onClick",
    dblclick: "onDoubleClick",
    wheel: "onWheel",
  },
  Wf = [
    "pointerenter",
    "pointerdown",
    "pointermove",
    "pointerup",
    "pointerleave",
    "click",
    "dblclick",
    "wheel",
  ],
  ls = ["pointermove", "pointerup"];
function ew({
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
      function a(c) {
        if (c.type === "pointerdown")
          for (const d of ls) window.addEventListener(d, a);
        else if (c.type === "pointerup")
          for (const d of ls) window.removeEventListener(d, a);
        const f = qv(c, s.current.axisContext, s.current.stateSeries, u);
        t.handleEvent(e, bv[c.type], f);
      }
      for (const c of Wf) u.addEventListener(c, a);
      return () => {
        for (const c of Wf) u.removeEventListener(c, a);
        for (const c of ls) window.removeEventListener(c, a);
      };
    }, [e, t]),
    h.jsx("rect", { ref: l, width: o, height: i, style: { fillOpacity: 0 } })
  );
}
function tw(e) {
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
    legendRef: c,
    plotId: f,
    annotations: d,
    tracking: x,
  } = e;
  return h.jsxs("svg", {
    xmlns: "http://www.w3.org/2000/svg",
    width: t,
    height: n,
    style: r,
    id: i ? `${i}-annotations` : void 0,
    className: o,
    children: [
      h.jsxs("g", {
        transform: `translate(${s}, ${l})`,
        children: [
          h.jsx("g", {
            style: { clipPath: `url(#seriesViewportClip-${f})` },
            children: d,
          }),
          x,
        ],
      }),
      h.jsx("g", {
        transform: `translate(${s}, ${l})`,
        children: h.jsx(Bp.Provider, {
          value: u,
          children: h.jsx("g", { ref: c, children: a }),
        }),
      }),
    ],
  });
}
function Vf(e) {
  const { style: t } = e;
  return h.jsx("rect", {
    ...e,
    style: { fillOpacity: t != null && t.fill ? void 0 : 0, ...t },
  });
}
function nw(e) {
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
    plotWidth: c,
    plotHeight: f,
    plotId: d,
    series: x,
    topAxisRef: g,
    topAxis: v,
    rightAxisRef: E,
    rightAxis: m,
    bottomAxisRef: p,
    bottomAxis: y,
    leftAxisRef: w,
    leftAxis: k,
    headingRef: S,
    heading: P,
  } = e;
  return h.jsxs("svg", {
    xmlns: "http://www.w3.org/2000/svg",
    width: t,
    height: n,
    style: r,
    id: i,
    className: o,
    children: [
      h.jsx(Vf, { width: t, height: n, style: l }),
      h.jsxs("g", {
        transform: `translate(${a}, ${u})`,
        children: [
          h.jsx(Vf, { width: c, height: f, style: s }),
          h.jsx("clipPath", {
            id: `seriesViewportClip-${d}`,
            children: h.jsx("rect", { width: c, height: f }),
          }),
          h.jsx("g", {
            style: { clipPath: `url(#seriesViewportClip-${d})` },
            children: x,
          }),
          h.jsx(qn.Provider, {
            value: g,
            children: h.jsx("g", { children: v }),
          }),
          h.jsx(qn.Provider, {
            value: E,
            children: h.jsx("g", { children: m }),
          }),
          h.jsx(qn.Provider, {
            value: p,
            children: h.jsx("g", { children: y }),
          }),
          h.jsx(qn.Provider, {
            value: w,
            children: h.jsx("g", { children: k }),
          }),
        ],
      }),
      h.jsx("g", { ref: S, children: P }),
    ],
  });
}
const rw = Cp($1),
  iw = {
    headingPosition: null,
    legendPosition: null,
    legendMargin: 0,
    series: [],
    axes: {},
  },
  ow = {
    overflow: "visible",
    fontFamily: "Arial, Helvetica, sans-serif",
    userSelect: "none",
    WebkitUserSelect: "none",
  },
  lw = { touchAction: "none" };
function e0(e) {
  const {
      width: t,
      height: n,
      colorScheme: r = l1,
      margin: i = {},
      svgStyle: o = {},
      svgId: l,
      svgClassName: s,
      plotViewportStyle: u = {},
      seriesViewportStyle: a = {},
      controllerId: c,
      children: f,
    } = e,
    d = kr(void 0, "plot"),
    [x, g] = _.useReducer(rw, iw, void 0),
    {
      series: v,
      annotations: E,
      topAxis: m,
      rightAxis: p,
      bottomAxis: y,
      leftAxis: w,
      heading: k,
      legend: S,
    } = Qv(f);
  if (t === void 0) throw new Error("width is mandatory");
  if (n === void 0) throw new Error("height is mandatory");
  const P = I1({ controllerId: c });
  _.useEffect(() => {
    if (P) return P.registerPlot(d), () => P.unregisterPlot(d);
  }, [P, d]);
  const C = R1({ controllerId: c }),
    D = ht(),
    O = ht(),
    N = ht(),
    H = ht(),
    R = ht(),
    Y = ht(),
    {
      plotWidth: K,
      plotHeight: Q,
      topOffset: oe,
      leftOffset: T,
      legendOffset: L,
    } = Xv({
      width: t,
      height: n,
      margin: i,
      axes: x.axes,
      topAxisHeight: O.height,
      rightAxisWidth: N.width,
      bottomAxisHeight: H.height,
      leftAxisWidth: R.width,
      headingPosition: x.headingPosition,
      headingHeight: D.height,
      legendPosition: x.legendPosition,
      legendMargin: x.legendMargin,
      legendWidth: Y.width,
      legendHeight: Y.height,
    }),
    z = A1(x, C.axes, { plotWidth: K, plotHeight: Q }),
    F = _.useMemo(() => x.series.map(({ id: tn }) => tn), [x.series]),
    X = _.useMemo(() => da().range(r).domain(F), [r, F]),
    Te = _.useMemo(
      () => ({
        width: t,
        height: n,
        plotWidth: K,
        plotHeight: Q,
        axisContext: z,
        colorScaler: X,
      }),
      [t, n, K, Q, z, X]
    ),
    Ge = {
      ...ow,
      ...(P ? lw : null),
      ...o,
      position: "absolute",
      top: "0",
      left: "0",
    };
  return h.jsx(Tp.Provider, {
    value: Te,
    children: h.jsx(_p.Provider, {
      value: g,
      children: h.jsx(M1, {
        children: h.jsxs("div", {
          style: { position: "relative", width: t, height: n },
          children: [
            h.jsx(nw, {
              width: t,
              height: n,
              svgStyle: Ge,
              svgId: l,
              svgClassName: s,
              plotViewportStyle: u,
              seriesViewportStyle: a,
              topOffset: oe,
              leftOffset: T,
              plotWidth: K,
              plotHeight: Q,
              plotId: d,
              series: v,
              topAxisRef: O.ref,
              topAxis: m,
              rightAxisRef: N.ref,
              rightAxis: p,
              bottomAxisRef: H.ref,
              bottomAxis: y,
              leftAxisRef: R.ref,
              leftAxis: w,
              headingRef: D.ref,
              heading: k,
            }),
            h.jsx(tw, {
              width: t,
              height: n,
              svgStyle: Ge,
              svgId: l,
              svgClassName: s,
              topOffset: oe,
              leftOffset: T,
              legendOffset: L,
              legend: S,
              legendRef: Y.ref,
              plotId: d,
              annotations: E,
              tracking: P
                ? h.jsx(ew, {
                    plotId: d,
                    plotEvents: P,
                    stateSeries: x.series,
                    axisContext: z,
                    plotWidth: K,
                    plotHeight: Q,
                  })
                : null,
            }),
          ],
        }),
      }),
    }),
  });
}
const eo = 1,
  Bf = 0.1;
function sw(e) {
  const t = e.robot.odometry.pose,
    n = e.odometryPlot,
    r = t.x + Bf * Math.cos(t.theta),
    i = t.y + Bf * Math.sin(t.theta);
  return h.jsxs(e0, {
    width: 500,
    height: 500,
    margin: { top: 20, right: 20, bottom: 20, left: 20 },
    children: [
      h.jsx(vl, { title: "Robot position in xy plane" }),
      h.jsx(wl, {
        displayMarkers: !0,
        markerShape: "circle",
        markerSize: 3,
        data: n.series[0],
      }),
      h.jsxs(Fp, {
        children: [
          h.jsx(Uf.Circle, { x: t.x, y: t.y, r: 0.03, color: "green" }),
          h.jsx(Uf.Line, {
            x1: t.x,
            y1: t.y,
            x2: r,
            y2: i,
            color: "blue",
            strokeWidth: 3,
          }),
        ],
      }),
      h.jsx(gn, {
        id: "x",
        position: "bottom",
        label: "x [m]",
        displayPrimaryGridLines: !0,
        min: -eo,
        max: eo,
      }),
      h.jsx(gn, {
        id: "y",
        position: "left",
        label: "y [m]",
        displayPrimaryGridLines: !0,
        min: -eo,
        max: eo,
      }),
    ],
  });
}
const ss = 3;
function uw(e) {
  const t = e.robot.odometry;
  return h.jsx("div", {
    children: h.jsxs("table", {
      children: [
        h.jsxs("tr", {
          children: [
            h.jsx("th", { align: "left", children: " Variable " }),
            h.jsx("th", { children: " Value" }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "x [m]" }),
            h.jsx("td", { align: "center", children: t.pose.x.toFixed(ss) }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "y [m]" }),
            h.jsx("td", { align: "center", children: t.pose.y.toFixed(ss) }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "theta [rad]" }),
            h.jsx("td", {
              align: "center",
              children: t.pose.theta.toFixed(ss),
            }),
          ],
        }),
      ],
    }),
  });
}
const Dr = 3;
function Yf(e) {
  return h.jsx("div", {
    children: h.jsxs("table", {
      children: [
        h.jsxs("tr", {
          children: [
            h.jsx("th", { align: "left", children: " Parameter " }),
            h.jsx("th", { children: " Value" }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "k_p" }),
            h.jsx("td", { align: "center", children: e.kp.toFixed(Dr) }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "k_i" }),
            h.jsx("td", { align: "center", children: e.ki.toFixed(Dr) }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "k_d" }),
            h.jsx("td", { align: "center", children: e.kd.toFixed(Dr) }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "Target value" }),
            h.jsx("td", { align: "center", children: e.target.toFixed(Dr) }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "Current value" }),
            h.jsx("td", { align: "center", children: e.current.toFixed(Dr) }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "Mode" }),
            h.jsx("td", {
              align: "center",
              children: e.mode ? "Enabled" : "Disabled",
            }),
          ],
        }),
      ],
    }),
  });
}
function zr(e) {
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
  return h.jsxs(e0, {
    width: a,
    height: u,
    margin: { top: 20, right: 20, bottom: 20, left: 20 },
    children: [
      h.jsx(Yp, { position: s }),
      h.jsx(vl, { title: r }),
      t.map((c, f) =>
        h.jsx(
          wl,
          {
            displayMarkers: !0,
            markerShape: "circle",
            markerSize: 5,
            data: c,
            label: n[f],
          },
          f
        )
      ),
      h.jsx(gn, {
        id: "x",
        position: "bottom",
        label: i,
        displayPrimaryGridLines: !0,
      }),
      h.jsx(gn, {
        id: "y",
        position: "left",
        label: o,
        displayPrimaryGridLines: !0,
        min: -l,
        max: l,
      }),
    ],
  });
}
const Gf = 0;
function aw(e) {
  return h.jsx("div", {
    children: h.jsxs("table", {
      children: [
        h.jsxs("tr", {
          children: [
            h.jsx("th", { align: "left", children: " Wheel " }),
            h.jsx("th", { children: " Speed [rpm]" }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "Left" }),
            h.jsx("td", {
              align: "center",
              children: e.robot.leftMotor.speed.toFixed(Gf),
            }),
          ],
        }),
        h.jsxs("tr", {
          children: [
            h.jsx("td", { children: "Right" }),
            h.jsx("td", {
              align: "center",
              children: e.robot.rightMotor.speed.toFixed(Gf),
            }),
          ],
        }),
      ],
    }),
  });
}
const Ai = 1e6;
function cw(e, t) {
  const { distancePlot: n } = e,
    { distances: r } = t,
    i = t.odometry.time,
    o = Sr(n.series, i, r, { xFactor: Ai, maxDataLength: Ke });
  return { ...n, series: o };
}
function fw(e, t) {
  const { odometryPlot: n } = e,
    { odometry: r } = t,
    i = Sr(n.series, r.pose.x, [r.pose.y], { maxDataLength: Ke });
  return { ...n, series: i };
}
function dw(e, t) {
  const { linearSpeedControllerPlot: n } = e,
    { controllers: r } = t,
    i = t.odometry.time,
    o = [r.v.target, r.v.current],
    l = Sr(n.series, i, o, { xFactor: Ai, maxDataLength: Ke });
  return { ...n, series: l };
}
function hw(e, t) {
  const { angularSpeedControllerPlot: n } = e,
    { controllers: r } = t,
    i = t.odometry.time,
    o = [r.omega.target, r.omega.current],
    l = Sr(n.series, i, o, { xFactor: Ai, maxDataLength: Ke });
  return { ...n, series: l };
}
function pw(e, t) {
  const { commandsPlot: n } = e,
    { leftMotor: r, rightMotor: i } = t,
    o = t.odometry.time,
    l = [r.command, i.command],
    s = Sr(n.series, o, l, { xFactor: Ai, maxDataLength: Ke });
  return { ...n, series: s };
}
function mw(e, t) {
  const { wheelSpeedsPlot: n } = e,
    { leftMotor: r, rightMotor: i } = t,
    o = t.odometry.time,
    l = [r.speed, i.speed],
    s = Sr(n.series, o, l, { xFactor: Ai, maxDataLength: Ke });
  return { ...n, series: s };
}
function Sr(e, t, n, r = {}) {
  const { xFactor: i = 1, yFactor: o = 1, maxDataLength: l = 100 } = r,
    s = e,
    u = s.length;
  for (let a = 0; a < u; a++) {
    let c = n[a];
    a > 2 && (c = -n[a]),
      (s[a] = [...s[a], { x: t / i, y: c / o }].slice(1, l + 1));
  }
  return s;
}
const Ke = 100,
  Qf = 5;
function yw() {
  const e = { target: 0, current: 0, kp: 0, ki: 0, kd: 0, mode: !0 };
  return {
    robot: {
      imu: {
        acceleration: { x: 0, y: 0, z: 0 },
        rotation: { x: 0, y: 0, z: 0 },
      },
      distances: new Array(Qf).fill(0),
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
      series: On(Qf, Ke),
      labels: ["Left", "Front-left", "Front", "Front-right", "Right"],
      xLabel: "Time [s]",
      yLabel: "Distance [mm]",
      title: "Distance sensors",
      yLimit: 1e3,
      legendPosition: "right",
      plotWidth: 800,
    },
    odometryPlot: { series: On(1, Ke), labels: ["Position"] },
    linearSpeedControllerPlot: {
      series: On(2, Ke),
      labels: ["Target speed", "Current speed"],
      yLimit: 0.5,
      yLabel: "v [m/s]",
      title: "Linear speed controller",
    },
    angularSpeedControllerPlot: {
      series: On(2, Ke),
      labels: ["Target speed", "Current speed"],
      yLabel: "omega [rad/s]",
      title: "Angular speed controller",
      yLimit: 5,
    },
    commandsPlot: {
      series: On(2, Ke),
      labels: ["Left", "Right"],
      xLabel: "Time [s]",
      yLabel: "Command [-]",
      title: "PWM commands applied to the motors",
      yLimit: 260,
    },
    wheelSpeedsPlot: {
      series: On(2, Ke),
      labels: ["Left", "Right"],
      xLabel: "Time [s]",
      yLabel: "Speed [rpm]",
      title: "Wheel speeds",
      yLimit: 600,
    },
  };
}
function On(e, t) {
  return new Array(e).fill(0).map(() => new Array(t).fill({ x: 0, y: 0 }));
}
function gw(e, t) {
  const n = JSON.parse(t);
  return {
    robot: n,
    maze: e.maze,
    distancePlot: cw(e, n),
    odometryPlot: fw(e, n),
    linearSpeedControllerPlot: dw(e, n),
    angularSpeedControllerPlot: hw(e, n),
    commandsPlot: pw(e, n),
    wheelSpeedsPlot: mw(e, n),
  };
}
const Vr = "";
function xw() {
  const [e, t] = _.useState(yw()),
    [n, r] = _.useState(""),
    [i, o] = _.useState("");
  return (
    _.useEffect(() => {
      const l = new EventSource(`${Vr}/events`);
      return (
        (l.onopen = () => {}),
        (l.onerror = (s) => {
          console.log("ERROR!", s);
        }),
        (l.onmessage = (s) => {}),
        l.addEventListener(
          "state",
          (s) => {
            const u = gw(e, s.data);
            t(u);
          },
          !1
        ),
        () => l.close()
      );
    }, [e, Vr]),
    h.jsxs("div", {
      style: { overflow: "clip" },
      children: [
        h.jsxs("div", {
          children: [
            "Command:",
            h.jsx("input", {
              type: "text",
              value: n,
              onChange: (l) => r(l.target.value),
              onKeyDown: async (l) => {
                l.key === "Enter" && (await yu(Vr, n, o));
              },
            }),
            h.jsx("button", { onClick: () => yu(Vr, n, o), children: "Send" }),
            h.jsx(us, { command: "h", label: "Help", setResponse: o }),
            h.jsx(us, { command: "ps", label: "State", setResponse: o }),
            h.jsx(us, { command: "s", label: "Settings", setResponse: o }),
          ],
        }),
        h.jsx("div", {
          children: h.jsx("textarea", { cols: 100, rows: 10, value: i }),
        }),
        h.jsx("h2", { children: "Accelerometer data" }),
        h.jsx($y, { ...e.robot.imu }),
        h.jsx("h2", { children: "Distance sensors" }),
        h.jsxs("div", {
          style: { display: "flex" },
          children: [
            h.jsx("div", { style: { flex: 1 }, children: h.jsx(Ny, { ...e }) }),
            h.jsx("div", {
              style: { flex: 1 },
              children: h.jsx(zr, { ...e.distancePlot }),
            }),
          ],
        }),
        h.jsx("h2", { children: "Odometry" }),
        h.jsxs("div", {
          style: { display: "flex" },
          children: [
            h.jsx("div", { style: { flex: 1 }, children: h.jsx(uw, { ...e }) }),
            h.jsx("div", { style: { flex: 1 }, children: h.jsx(sw, { ...e }) }),
          ],
        }),
        h.jsx("h2", { children: "Robot speed controllers" }),
        h.jsx("h3", { children: "Linear speed" }),
        h.jsxs("div", {
          style: { display: "flex" },
          children: [
            h.jsx("div", {
              style: { flex: 1 },
              children: h.jsx(Yf, { ...e.robot.controllers.v }),
            }),
            h.jsx("div", {
              style: { flex: 1 },
              children: h.jsx(zr, { ...e.linearSpeedControllerPlot }),
            }),
          ],
        }),
        h.jsx("h3", { children: "Angular speed" }),
        h.jsxs("div", {
          style: { display: "flex" },
          children: [
            h.jsx("div", {
              style: { flex: 1 },
              children: h.jsx(Yf, { ...e.robot.controllers.omega }),
            }),
            h.jsx("div", {
              style: { flex: 1 },
              children: h.jsx(zr, { ...e.angularSpeedControllerPlot }),
            }),
          ],
        }),
        h.jsx("h2", { children: "Motors PWM commands" }),
        h.jsxs("div", {
          style: { display: "flex" },
          children: [
            h.jsx("div", { style: { flex: 1 }, children: h.jsx(Ly, { ...e }) }),
            h.jsx("div", {
              style: { flex: 1 },
              children: h.jsx(zr, { ...e.commandsPlot }),
            }),
          ],
        }),
        h.jsx("h2", { children: "Wheels speed" }),
        h.jsxs("div", {
          style: { display: "flex" },
          children: [
            h.jsx("div", { style: { flex: 1 }, children: h.jsx(aw, { ...e }) }),
            h.jsx("div", {
              style: { flex: 1 },
              children: h.jsx(zr, { ...e.wheelSpeedsPlot }),
            }),
          ],
        }),
      ],
    })
  );
}
async function yu(e, t, n) {
  const i = await (await fetch(`${e}/command?value=${t}`)).text();
  n == null || n(i);
}
function us(e) {
  const { command: t, label: n, setResponse: r } = e;
  return h.jsx("button", { onClick: () => yu(Vr, t, r), children: n || t });
}
as.createRoot(document.getElementById("root")).render(
  h.jsxs(x0.StrictMode, {
    children: [h.jsx("h1", { children: "Algernon debug page" }), h.jsx(xw, {})],
  })
);
