import { EventEmitter as ke } from "eventemitter3";
import { v4 as nt } from "uuid";
import { deserialize as $t } from "bson";
import { decode as jt } from "cbor2";
import { decode as Wt } from "fast-png";
function ut(n) {
  return n instanceof Object && "op" in n && typeof n.op == "string";
}
function Qt(n) {
  return n.op === "status";
}
function _r(n) {
  return n.op === "set_level";
}
function Jt(n) {
  return n.op === "fragment";
}
function Zt(n) {
  return n.op === "png";
}
function Sr(n) {
  return n.op === "advertise";
}
function Or(n) {
  return n.op === "unadvertise";
}
function Bt(n) {
  return n.op === "publish";
}
function Rr(n) {
  return n.op === "subscribe";
}
function Mr(n) {
  return n.op === "unsubscribe";
}
function xr(n) {
  return n.op === "advertise_service";
}
function Ir(n) {
  return n.op === "unadvertise_service";
}
function At(n) {
  return n.op === "call_service";
}
function Ft(n) {
  return n.op === "service_response";
}
function Br(n) {
  return n.op === "advertise_action";
}
function Fr(n) {
  return n.op === "unadvertise_action";
}
function Lt(n) {
  return n.op === "send_action_goal";
}
function Pt(n) {
  return n.op === "cancel_action_goal";
}
function kt(n) {
  return n.op === "action_feedback";
}
function Ut(n) {
  return n.op === "action_result";
}
class re extends ke {
  /**
   * @param options
   * @param options.ros - The ROSLIB.Ros connection handle.
   * @param options.name - The service name, like '/add_two_ints'.
   * @param options.serviceType - The service type, like 'rospy_tutorials/AddTwoInts'.
   */
  constructor({
    ros: t,
    name: u,
    serviceType: s
  }) {
    super(), this.#e = null, this.isAdvertised = !1, this.#t = Promise.resolve(), this.#r = !1, this.ros = t, this.name = u, this.serviceType = s;
  }
  #e;
  #t;
  #r;
  /**
   * Call the service. Returns the service response in the
   * callback. Does nothing if this service is currently advertised.
   *
   * @param request - The service request to send.
   * @param [callback] - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   * @param [timeout] - Optional timeout, in seconds, for the service call. A non-positive value means no timeout.
   *                             If not provided, the rosbridge server will use its default value.
   */
  callService(t, u, s = console.error, c) {
    if (this.isAdvertised)
      return;
    const o = `call_service:${this.name}:${nt()}`;
    this.ros.once(o, function(h) {
      Ft(h) && (h.result ? u?.(h.values) : s(h.values ?? ""));
    }), this.ros.callOnConnection({
      op: "call_service",
      id: o,
      service: this.name,
      args: t,
      timeout: c
    });
  }
  /**
   * Advertise the service. This turns the Service object from a client
   * into a server. The callback will be called with every request
   * that's made on this service.
   *
   * @param callback This works similarly to the callback for a C++ service in that you should take care not to overwrite the response object.
   *  Instead, only modify the values within.
   */
  async advertise(t) {
    return this.#t = this.#t.then(() => {
      this.isAdvertised && this.#u(), this.#e = (u) => {
        if (!At(u))
          throw new Error(
            `Invalid message received on service channel: ${JSON.stringify(u)}`
          );
        const s = {};
        let c;
        try {
          c = t(u.args, s);
        } catch {
          c = !1;
        }
        c ? this.ros.callOnConnection({
          op: "service_response",
          service: this.name,
          values: s,
          result: c,
          id: u.id
        }) : this.ros.callOnConnection({
          op: "service_response",
          service: this.name,
          result: c,
          id: u.id
        });
      }, this.ros.on(this.name, this.#e), this.ros.callOnConnection({
        op: "advertise_service",
        type: this.serviceType,
        service: this.name
      }), this.isAdvertised = !0;
    }).catch((u) => {
      throw this.emit("error", u), u;
    }), this.#t;
  }
  /**
   * Internal method to perform unadvertisement without queueing
   */
  #u() {
    if (!(!this.isAdvertised || this.#r)) {
      this.#r = !0;
      try {
        this.isAdvertised = !1, this.#e && (this.ros.off(this.name, this.#e), this.#e = null), this.ros.callOnConnection({
          op: "unadvertise_service",
          service: this.name
        });
      } finally {
        this.#r = !1;
      }
    }
  }
  async unadvertise() {
    return this.#t = this.#t.then(() => {
      this.#u();
    }).catch((t) => {
      throw this.emit("error", t), t;
    }), this.#t;
  }
  /**
   * An alternate form of Service advertisement that supports a modern Promise-based interface for use with async/await.
   * @param callback An asynchronous callback processing the request and returning a response.
   */
  async advertiseAsync(t) {
    return this.#t = this.#t.then(() => {
      this.isAdvertised && this.#u(), this.#e = (u) => {
        if (!At(u))
          throw new Error(
            `Invalid message received on service channel: ${JSON.stringify(u)}`
          );
        (async () => {
          try {
            this.ros.callOnConnection({
              op: "service_response",
              service: this.name,
              result: !0,
              values: await t(u.args),
              id: u.id
            });
          } catch (s) {
            this.ros.callOnConnection({
              op: "service_response",
              service: this.name,
              result: !1,
              values: String(s),
              id: u.id
            });
          }
        })().catch(console.error);
      }, this.ros.on(this.name, this.#e), this.ros.callOnConnection({
        op: "advertise_service",
        type: this.serviceType,
        service: this.name
      }), this.isAdvertised = !0;
    }).catch((u) => {
      throw this.emit("error", u), u;
    }), this.#t;
  }
}
class fe extends ke {
  /**
   * @param options
   * @param options.ros - The ROSLIB.Ros connection handle.
   * @param options.name - The topic name, like '/cmd_vel'.
   * @param options.messageType - The message type, like 'std_msgs/String'.
   * @param [options.compression=none] - The type of compression to use, like 'png', 'cbor', or 'cbor-raw'.
   * @param [options.throttle_rate=0] - The rate (in ms in between messages) at which to throttle the topics.
   * @param [options.queue_size=100] - The queue created at bridge side for re-publishing webtopics.
   * @param [options.latch=false] - Latch the topic when publishing.
   * @param [options.queue_length=0] - The queue length at bridge side used when subscribing.
   * @param [options.reconnect_on_close=true] - The flag to enable resubscription and readvertisement on close event.
   */
  constructor({
    ros: t,
    name: u,
    messageType: s,
    compression: c = "none",
    throttle_rate: o = 0,
    latch: h = !1,
    queue_size: C = 100,
    queue_length: f = 0,
    reconnect_on_close: g = !0
  }) {
    super(), this.waitForReconnect = !1, this.reconnectFunc = void 0, this.isAdvertised = !1, this.subscribeId = null, this.#e = (D) => {
      if (Bt(D))
        this.emit("message", D.msg);
      else
        throw new Error(
          `Unexpected message on topic channel: ${JSON.stringify(D)}`
        );
    }, this.ros = t, this.name = u, this.messageType = s, this.compression = c, this.throttle_rate = o, this.latch = h, this.queue_size = C, this.queue_length = f, this.reconnect_on_close = g, this.compression && this.compression !== "png" && this.compression !== "cbor" && this.compression !== "cbor-raw" && this.compression !== "none" && (this.emit(
      "warning",
      `${this.compression} compression is not supported. No compression will be used.`
    ), this.compression = "none"), this.throttle_rate < 0 && (this.emit(
      "warning",
      `${this.throttle_rate.toString()} is not allowed. Set to 0`
    ), this.throttle_rate = 0), this.reconnect_on_close ? this.callForSubscribeAndAdvertise = (D) => {
      this.ros.callOnConnection(D), this.waitForReconnect = !1, this.reconnectFunc = () => {
        this.waitForReconnect || (this.waitForReconnect = !0, this.ros.callOnConnection(D), this.ros.once("connection", () => {
          this.waitForReconnect = !1;
        }));
      }, this.ros.on("close", this.reconnectFunc);
    } : this.callForSubscribeAndAdvertise = (D) => {
      this.ros.callOnConnection(D);
    };
  }
  #e;
  /**
   * Every time a message is published for the given topic, the callback
   * will be called with the message object.
   *
   * @param callback - Function with the following params:
   */
  subscribe(t) {
    this.on("message", t), !this.subscribeId && (this.ros.on(this.name, this.#e), this.subscribeId = `subscribe:${this.name}:${nt()}`, this.callForSubscribeAndAdvertise({
      op: "subscribe",
      id: this.subscribeId,
      type: this.messageType,
      topic: this.name,
      compression: this.compression,
      throttle_rate: this.throttle_rate,
      queue_length: this.queue_length
    }));
  }
  /**
   * Unregister as a subscriber for the topic. Unsubscribing will stop
   * and remove all subscribe callbacks. To remove a callback, you must
   * explicitly pass the callback function in.
   *
   * @param [callback] - The callback to unregister, if
   *     provided and other listeners are registered the topic won't
   *     unsubscribe, just stop emitting to the passed listener.
   */
  unsubscribe(t) {
    t && (this.off("message", t), this.listeners("message").length) || this.subscribeId && (this.ros.off(this.name, this.#e), this.reconnect_on_close && this.ros.off("close", this.reconnectFunc), this.emit("unsubscribe"), this.ros.callOnConnection({
      op: "unsubscribe",
      id: this.subscribeId,
      topic: this.name
    }), this.subscribeId = null);
  }
  /**
   * Register as a publisher for the topic.
   */
  advertise() {
    this.isAdvertised || (this.advertiseId = `advertise:${this.name}:${nt()}`, this.callForSubscribeAndAdvertise({
      op: "advertise",
      id: this.advertiseId,
      type: this.messageType,
      topic: this.name,
      latch: this.latch,
      queue_size: this.queue_size
    }), this.isAdvertised = !0, this.reconnect_on_close || this.ros.on("close", () => {
      this.isAdvertised = !1;
    }));
  }
  /**
   * Unregister as a publisher for the topic.
   */
  unadvertise() {
    this.isAdvertised && (this.reconnect_on_close && this.ros.off("close", this.reconnectFunc), this.emit("unadvertise"), this.ros.callOnConnection({
      op: "unadvertise",
      id: this.advertiseId,
      topic: this.name
    }), this.isAdvertised = !1);
  }
  /**
   * Publish the message.
   *
   * @param message - The message to publish.
   */
  publish(t) {
    this.isAdvertised || this.advertise(), this.ros.callOnConnection({
      op: "publish",
      id: `publish:${this.name}:${nt()}`,
      topic: this.name,
      msg: t
    });
  }
  /**
   * Retrieves list of publishers for this topic.
   *
   * @param callback - Function with the following params:
   *   * publishers - The list of publishers.
   * @param [failedCallback] - The callback function when the service call failed.
   */
  getPublishers(t, u = console.error) {
    const s = new re({
      ros: this.ros,
      name: "/rosapi/publishers",
      serviceType: "rosapi/Publishers"
    }), c = {
      topic: this.name
    };
    s.callService(
      c,
      function(o) {
        t(o.publishers);
      },
      function(o) {
        u(o);
      }
    );
  }
}
class Kt {
  /**
   * @param options
   * @param options.ros - The ROSLIB.Ros connection handle.
   * @param options.name - The param name, like max_vel_x.
   */
  constructor({ ros: t, name: u }) {
    this.ros = t, this.name = u;
  }
  /**
   * Fetch the value of the param.
   *
   * @param callback - The callback function.
   * @param [failedCallback] - The callback function when the service call failed or the parameter retrieval was unsuccessful.
   */
  get(t, u = console.error) {
    const s = new re({
      ros: this.ros,
      name: "rosapi/get_param",
      serviceType: "rosapi/GetParam"
    }), c = { name: this.name };
    s.callService(
      c,
      function(o) {
        "successful" in o && !o.successful ? u(o.reason) : t(JSON.parse(o.value));
      },
      u
    );
  }
  /**
   * Set the value of the param in ROS.
   *
   * @param value - The value to set param to.
   * @param [callback] - The callback function.
   * @param [failedCallback] - The callback function when the service call failed or the parameter setting was unsuccessful.
   */
  set(t, u, s = console.error) {
    const c = new re({
      ros: this.ros,
      name: "rosapi/set_param",
      serviceType: "rosapi/SetParam"
    }), o = {
      name: this.name,
      value: JSON.stringify(t)
    };
    c.callService(
      o,
      function(h) {
        "successful" in h && !h.successful ? s(h.reason) : u && u(h);
      },
      s
    );
  }
  /**
   * Delete this parameter on the ROS server.
   *
   * @param callback - The callback function.
   * @param [failedCallback] - The callback function when the service call failed or the parameter deletion was unsuccessful.
   */
  delete(t, u = console.error) {
    const s = new re({
      ros: this.ros,
      name: "rosapi/delete_param",
      serviceType: "rosapi/DeleteParam"
    }), c = {
      name: this.name
    };
    s.callService(
      c,
      function(o) {
        "successful" in o && !o.successful ? u(o.reason) : t(o);
      },
      u
    );
  }
}
class qt extends ke {
  /**
   * @param options
   * @param options.ros - The ROSLIB.Ros connection handle.
   * @param options.serverName - The action server name, like '/fibonacci'.
   * @param options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
   * @param [options.timeout] - The timeout length when connecting to the action server.
   * @param [options.omitFeedback] - The flag to indicate whether to omit the feedback channel or not.
   * @param [options.omitStatus] - The flag to indicate whether to omit the status channel or not.
   * @param [options.omitResult] - The flag to indicate whether to omit the result channel or not.
   */
  constructor({
    ros: t,
    serverName: u,
    actionName: s,
    timeout: c,
    omitFeedback: o,
    omitStatus: h,
    omitResult: C
  }) {
    super(), this.goals = {}, this.receivedStatus = !1, this.ros = t, this.serverName = u, this.actionName = s, this.timeout = c, this.omitFeedback = o, this.omitStatus = h, this.omitResult = C, this.feedbackListener = new fe({
      ros: this.ros,
      name: `${this.serverName}/feedback`,
      messageType: `${this.actionName}Feedback`
    }), this.statusListener = new fe({
      ros: this.ros,
      name: `${this.serverName}/status`,
      messageType: "actionlib_msgs/GoalStatusArray"
    }), this.resultListener = new fe({
      ros: this.ros,
      name: `${this.serverName}/result`,
      messageType: `${this.actionName}Result`
    }), this.goalTopic = new fe({
      ros: this.ros,
      name: `${this.serverName}/goal`,
      messageType: `${this.actionName}Goal`
    }), this.cancelTopic = new fe({
      ros: this.ros,
      name: `${this.serverName}/cancel`,
      messageType: "actionlib_msgs/GoalID"
    }), this.goalTopic.advertise(), this.cancelTopic.advertise(), this.omitStatus || this.statusListener.subscribe((f) => {
      this.receivedStatus = !0, f.status_list.forEach((g) => {
        const D = this.goals[g.goal_id.id];
        D && D.emit("status", g);
      });
    }), this.omitFeedback || this.feedbackListener.subscribe((f) => {
      const g = this.goals[f.status.goal_id.id];
      g && (g.emit("status", f.status), g.emit("feedback", f.feedback));
    }), this.omitResult || this.resultListener.subscribe((f) => {
      const g = this.goals[f.status.goal_id.id];
      g && (g.emit("status", f.status), g.emit("result", f.result));
    }), this.timeout && setTimeout(() => {
      this.receivedStatus || this.emit("timeout");
    }, this.timeout);
  }
  /**
   * Cancel all goals associated with this ActionClient.
   */
  cancel() {
    const t = {};
    this.cancelTopic.publish(t);
  }
  /**
   * Unsubscribe and unadvertise all topics associated with this ActionClient.
   */
  dispose() {
    this.goalTopic.unadvertise(), this.cancelTopic.unadvertise(), this.omitStatus || this.statusListener.unsubscribe(), this.omitFeedback || this.feedbackListener.unsubscribe(), this.omitResult || this.resultListener.unsubscribe();
  }
}
class er extends ke {
  /**
   * @param options
   * @param options.actionClient - The ROSLIB.ActionClient to use with this goal.
   * @param options.goalMessage - The JSON object containing the goal for the action server.
   */
  constructor({
    actionClient: t,
    goalMessage: u
  }) {
    super(), this.isFinished = !1, this.status = void 0, this.result = void 0, this.feedback = void 0, this.goalID = `goal_${nt()}`, this.actionClient = t, this.goalMessage = {
      goal_id: {
        stamp: {
          secs: 0,
          nsecs: 0
        },
        id: this.goalID
      },
      goal: u
    }, this.on("status", (s) => {
      this.status = s;
    }), this.on("result", (s) => {
      this.isFinished = !0, this.result = s;
    }), this.on("feedback", (s) => {
      this.feedback = s;
    }), this.actionClient.goals[this.goalID] = this;
  }
  /**
   * Send the goal to the action server.
   *
   * @param [timeout] - A timeout length for the goal's result.
   */
  send(t) {
    this.actionClient.goalTopic.publish(this.goalMessage), t && setTimeout(() => {
      this.isFinished || this.emit("timeout");
    }, t);
  }
  /**
   * Cancel the current goal.
   */
  cancel() {
    const t = {
      id: this.goalID
    };
    this.actionClient.cancelTopic.publish(t);
  }
}
class Oe {
  constructor(t) {
    this.x = t?.x ?? 0, this.y = t?.y ?? 0, this.z = t?.z ?? 0;
  }
  /**
   * Set the values of this vector to the sum of itself and the given vector.
   *
   * @param v - The vector to add with.
   */
  add(t) {
    this.x += t.x, this.y += t.y, this.z += t.z;
  }
  /**
   * Set the values of this vector to the difference of itself and the given vector.
   *
   * @param v - The vector to subtract with.
   */
  subtract(t) {
    this.x -= t.x, this.y -= t.y, this.z -= t.z;
  }
  /**
   * Multiply the given Quaternion with this vector.
   *
   * @param q - The quaternion to multiply with.
   */
  multiplyQuaternion(t) {
    const u = t.w * this.x + t.y * this.z - t.z * this.y, s = t.w * this.y + t.z * this.x - t.x * this.z, c = t.w * this.z + t.x * this.y - t.y * this.x, o = -t.x * this.x - t.y * this.y - t.z * this.z;
    this.x = u * t.w + o * -t.x + s * -t.z - c * -t.y, this.y = s * t.w + o * -t.y + c * -t.x - u * -t.z, this.z = c * t.w + o * -t.z + u * -t.y - s * -t.x;
  }
  /**
   * Clone a copy of this vector.
   *
   * @returns The cloned vector.
   */
  clone() {
    return new Oe(this);
  }
}
class Qe {
  constructor(t) {
    this.x = t?.x ?? 0, this.y = t?.y ?? 0, this.z = t?.z ?? 0, this.w = typeof t?.w == "number" ? t.w : 1;
  }
  /**
   * Perform a conjugation on this quaternion.
   */
  conjugate() {
    this.x *= -1, this.y *= -1, this.z *= -1;
  }
  /**
   * Return the norm of this quaternion.
   */
  norm() {
    return Math.sqrt(
      this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w
    );
  }
  /**
   * Perform a normalization on this quaternion.
   */
  normalize() {
    let t = Math.sqrt(
      this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w
    );
    t === 0 ? (this.x = 0, this.y = 0, this.z = 0, this.w = 1) : (t = 1 / t, this.x = this.x * t, this.y = this.y * t, this.z = this.z * t, this.w = this.w * t);
  }
  /**
   * Convert this quaternion into its inverse.
   */
  invert() {
    this.conjugate(), this.normalize();
  }
  /**
   * Set the values of this quaternion to the product of itself and the given quaternion.
   *
   * @param q - The quaternion to multiply with.
   */
  multiply(t) {
    const u = this.x * t.w + this.y * t.z - this.z * t.y + this.w * t.x, s = -this.x * t.z + this.y * t.w + this.z * t.x + this.w * t.y, c = this.x * t.y - this.y * t.x + this.z * t.w + this.w * t.z, o = -this.x * t.x - this.y * t.y - this.z * t.z + this.w * t.w;
    this.x = u, this.y = s, this.z = c, this.w = o;
  }
  /**
   * Clone a copy of this quaternion.
   *
   * @returns The cloned quaternion.
   */
  clone() {
    return new Qe(this);
  }
}
class vt {
  constructor(t) {
    this.translation = new Oe(t.translation), this.rotation = new Qe(t.rotation);
  }
  /**
   * Clone a copy of this transform.
   *
   * @returns The cloned transform.
   */
  clone() {
    return new vt(this);
  }
}
class Gt {
  /**
   * @param options
   * @param options.ros - The ROSLIB.Ros connection handle.
   * @param [options.fixedFrame=base_link] - The fixed frame.
   * @param [options.angularThres=2.0] - The angular threshold for the TF republisher.
   * @param [options.transThres=0.01] - The translation threshold for the TF republisher.
   * @param [options.rate=10.0] - The rate for the TF republisher.
   * @param [options.updateDelay=50] - The time (in ms) to wait after a new subscription
   *     to update the TF republisher's list of TFs.
   * @param [options.topicTimeout=2.0] - The timeout parameter for the TF republisher.
   * @param [options.serverName="/tf2_web_republisher"] - The name of the tf2_web_republisher server.
   */
  constructor({
    ros: t,
    fixedFrame: u = "base_link",
    angularThres: s = 2,
    transThres: c = 0.01,
    rate: o = 10,
    updateDelay: h = 50,
    topicTimeout: C = 2,
    serverName: f = "/tf2_web_republisher"
  }) {
    this.frameInfos = {}, this.republisherUpdateRequested = !1, this.ros = t, this.fixedFrame = u, this.angularThres = s, this.transThres = c, this.rate = o, this.updateDelay = h;
    const g = C, D = Math.floor(g), p = Math.floor((g - D) * 1e9);
    this.topicTimeout = {
      secs: D,
      nsecs: p
    }, this.serverName = f;
  }
  /**
   * Process the incoming TF message and send them out using the callback
   * functions.
   *
   * @param tf - The TF message from the server.
   */
  processTFArray(t) {
    t.transforms.forEach((u) => {
      let s = u.child_frame_id;
      s.startsWith("/") && (s = s.substring(1));
      const c = this.frameInfos[s];
      if (c) {
        const o = new vt({
          translation: u.transform.translation,
          rotation: u.transform.rotation
        });
        c.transform = o, c.cbs.forEach((h) => {
          h(o);
        });
      }
    }, this);
  }
  /**
   * Create and send a new goal (or service request) to the tf2_web_republisher
   * based on the current list of TFs.
   * This method should be overridden by subclasses.
   */
  updateGoal() {
    throw new Error("updateGoal() must be implemented by subclass");
  }
  /**
   * Subscribe to the given TF frame.
   *
   * @param frameID - The TF frame to subscribe to.
   * @param callback - Function with the following params:
   */
  subscribe(t, u) {
    t.startsWith("/") && (t = t.substring(1)), this.frameInfos[t] || (this.frameInfos[t] = {
      cbs: []
    }, this.republisherUpdateRequested || (setTimeout(() => {
      this.updateGoal();
    }, this.updateDelay), this.republisherUpdateRequested = !0));
    const s = this.frameInfos[t]?.transform;
    s && u(s), this.frameInfos[t]?.cbs.push(u);
  }
  /**
   * Unsubscribe from the given TF frame.
   *
   * @param frameID - The TF frame to unsubscribe from.
   * @param [callback] - The callback function to remove.
   */
  unsubscribe(t, u) {
    t.startsWith("/") && (t = t.substring(1));
    const s = this.frameInfos[t];
    for (var c = s?.cbs ?? [], o = c.length; o--; )
      c[o] === u && c.splice(o, 1);
    (!u || c.length === 0) && delete this.frameInfos[t];
  }
}
class tr extends Gt {
  /**
   * @param options
   * @param options.ros - The ROSLIB.Ros connection handle.
   * @param [options.fixedFrame=base_link] - The fixed frame.
   * @param [options.angularThres=2.0] - The angular threshold for the TF republisher.
   * @param [options.transThres=0.01] - The translation threshold for the TF republisher.
   * @param [options.rate=10.0] - The rate for the TF republisher.
   * @param [options.updateDelay=50] - The time (in ms) to wait after a new subscription
   *     to update the TF republisher's list of TFs.
   * @param [options.topicTimeout=2.0] - The timeout parameter for the TF republisher.
   * @param [options.serverName="/tf2_web_republisher"] - The name of the tf2_web_republisher server.
   */
  constructor(t) {
    super(t), this.currentGoal = !1, this.currentTopic = !1, this.#e = void 0, this.#t = !1, this.actionClient = new qt({
      ros: this.ros,
      serverName: this.serverName,
      actionName: "tf2_web_republisher/TFSubscriptionAction",
      omitStatus: !0,
      omitResult: !0
    });
  }
  #e;
  #t;
  /**
   * Create and send a new goal (or service request) to the tf2_web_republisher
   * based on the current list of TFs.
   */
  updateGoal() {
    const t = {
      source_frames: Object.keys(this.frameInfos),
      target_frame: this.fixedFrame,
      angular_thres: this.angularThres,
      trans_thres: this.transThres,
      rate: this.rate
    };
    this.currentGoal && this.currentGoal.cancel(), this.currentGoal = new er({
      actionClient: this.actionClient,
      goalMessage: t
    }), this.currentGoal.on("feedback", (u) => {
      this.processTFArray(u);
    }), this.currentGoal.send(), this.republisherUpdateRequested = !1;
  }
  /**
   * Process the service response and subscribe to the tf republisher
   * topic.
   *
   * @param response - The service response containing the topic name.
   */
  processResponse(t) {
    this.#t || (this.currentTopic && this.currentTopic.unsubscribe(this.#e), this.currentTopic = new fe({
      ros: this.ros,
      name: t.topic_name,
      messageType: "tf2_web_republisher/TFArray"
    }), this.#e = (u) => {
      this.processTFArray(u);
    }, this.currentTopic.subscribe(this.#e));
  }
  /**
   * Unsubscribe and unadvertise all topics associated with this TFClient.
   */
  dispose() {
    this.#t = !0, this.actionClient.dispose(), this.currentTopic && this.currentTopic.unsubscribe(this.#e);
  }
}
class rr extends ke {
  /**
   * @param options
   * @param options.ros - The ROSLIB.Ros connection handle.
   * @param options.serverName - The action server name, like '/fibonacci'.
   * @param options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
   */
  constructor({
    ros: t,
    serverName: u,
    actionName: s
  }) {
    super(), this.currentGoal = null, this.nextGoal = null, this.ros = t, this.serverName = u, this.actionName = s, this.feedbackPublisher = new fe({
      ros: this.ros,
      name: `${this.serverName}/feedback`,
      messageType: `${this.actionName}Feedback`
    }), this.feedbackPublisher.advertise();
    const c = new fe({
      ros: this.ros,
      name: `${this.serverName}/status`,
      messageType: "actionlib_msgs/GoalStatusArray"
    });
    c.advertise(), this.resultPublisher = new fe({
      ros: this.ros,
      name: `${this.serverName}/result`,
      messageType: `${this.actionName}Result`
    }), this.resultPublisher.advertise();
    const o = new fe({
      ros: this.ros,
      name: `${this.serverName}/goal`,
      messageType: `${this.actionName}Goal`
    }), h = new fe({
      ros: this.ros,
      name: `${this.serverName}/cancel`,
      messageType: "actionlib_msgs/GoalID"
    });
    this.statusMessage = {
      header: {
        stamp: { secs: 0, nsecs: 100 },
        frame_id: ""
      },
      /** @type {{goal_id: any, status: number}[]} */
      status_list: []
    }, o.subscribe((f) => {
      this.currentGoal ? (this.nextGoal = f, this.emit("cancel")) : (this.statusMessage.status_list = [
        { goal_id: f.goal_id, status: 1 }
      ], this.currentGoal = f, this.emit("goal", f.goal));
    });
    const C = function(f, g) {
      return f.secs > g.secs ? !1 : f.secs < g.secs ? !0 : f.nsecs < g.nsecs;
    };
    h.subscribe((f) => {
      f.stamp.secs === 0 && f.stamp.nsecs === 0 && f.id === "" ? (this.nextGoal = null, this.currentGoal && this.emit("cancel")) : (this.currentGoal && f.id === this.currentGoal.goal_id.id ? this.emit("cancel") : this.nextGoal && f.id === this.nextGoal.goal_id.id && (this.nextGoal = null), this.nextGoal && C(this.nextGoal.goal_id.stamp, f.stamp) && (this.nextGoal = null), this.currentGoal && C(this.currentGoal.goal_id.stamp, f.stamp) && this.emit("cancel"));
    }), setInterval(() => {
      const f = /* @__PURE__ */ new Date(), g = Math.floor(f.getTime() / 1e3), D = Math.round(
        1e9 * (f.getTime() / 1e3 - g)
      );
      this.statusMessage.header = {
        ...this.statusMessage.header,
        stamp: { secs: g, nsecs: D }
      }, c.publish(this.statusMessage);
    }, 500);
  }
  /**
   * Set action state to succeeded and return to client.
   *
   * @param result - The result to return to the client.
   */
  setSucceeded(t) {
    if (this.currentGoal !== null) {
      const u = {
        status: { goal_id: this.currentGoal.goal_id, status: 3 },
        result: t
      };
      this.resultPublisher.publish(u), this.statusMessage.status_list = [], this.nextGoal ? (this.currentGoal = this.nextGoal, this.nextGoal = null, this.emit("goal", this.currentGoal.goal)) : this.currentGoal = null;
    }
  }
  /**
   * Set action state to aborted and return to client.
   *
   * @param result - The result to return to the client.
   */
  setAborted(t) {
    if (this.currentGoal !== null) {
      const u = {
        status: { goal_id: this.currentGoal.goal_id, status: 4 },
        result: t
      };
      this.resultPublisher.publish(u), this.statusMessage.status_list = [], this.nextGoal ? (this.currentGoal = this.nextGoal, this.nextGoal = null, this.emit("goal", this.currentGoal.goal)) : this.currentGoal = null;
    }
  }
  /**
   * Send a feedback message.
   *
   * @param feedback - The feedback to send to the client.
   */
  sendFeedback(t) {
    if (this.currentGoal !== null) {
      const u = {
        status: { goal_id: this.currentGoal.goal_id, status: 1 },
        feedback: t
      };
      this.feedbackPublisher.publish(u);
    }
  }
  /**
   * Handle case where client requests preemption.
   */
  setPreempted() {
    if (this.currentGoal !== null) {
      this.statusMessage.status_list = [];
      const t = {
        status: { goal_id: this.currentGoal.goal_id, status: 2 }
      };
      this.resultPublisher.publish(t), this.nextGoal ? (this.currentGoal = this.nextGoal, this.nextGoal = null, this.emit("goal", this.currentGoal.goal)) : this.currentGoal = null;
    }
  }
}
const ur = async (n) => {
  if (typeof WebSocket == "function") {
    const o = await import("./NativeWebSocketTransport-CF_ebnyS.js"), { NativeWebSocketTransport: h } = o, C = new WebSocket(n);
    return C.binaryType = "arraybuffer", new h(C);
  }
  const t = await import("ws"), u = await import("./WsWebSocketTransport-6-v9C0gj.js"), { WsWebSocketTransport: s } = u, c = new t.WebSocket(n);
  return c.binaryType = "arraybuffer", new s(c);
};
class Lr extends ke {
  // private write, public read via getter method
  #e;
  constructor({
    url: t,
    transportFactory: u = ur
  } = {}) {
    super(), this.#e = !1, this.transportFactory = u, t && this.connect(t).catch(console.error);
  }
  get isConnected() {
    return this.#e;
  }
  async connect(t) {
    if (this.transport && !this.transport.isClosed())
      return;
    const u = await this.transportFactory(t);
    this.transport = u, u.on("open", (s) => {
      this.#e = !0, this.emit("connection", s);
    }), u.on("close", (s) => {
      this.#e = !1, this.emit("close", s);
    }), u.on("error", (s) => {
      this.emit("error", s);
    }), u.on("message", (s) => {
      this.handleMessage(s);
    });
  }
  close() {
    this.transport?.close();
  }
  handleMessage(t) {
    Bt(t) ? this.emit(t.topic, t) : Ft(t) ? t.id ? this.emit(t.id, t) : console.error("Received service response without ID") : At(t) ? this.emit(t.service, t) : Lt(t) ? this.emit(t.action, t) : Pt(t) ? this.emit(t.id, t) : kt(t) ? this.emit(t.id, t) : Ut(t) ? this.emit(t.id, t) : Qt(t) && (t.id ? this.emit(`status:${t.id}`, t) : this.emit("status", t));
  }
  /**
   * Send an authorization request to the server.
   *
   * @param mac - MAC (hash) string given by the trusted source.
   * @param client - IP of the client.
   * @param dest - IP of the destination.
   * @param rand - Random string given by the trusted source.
   * @param t - Time of the authorization request.
   * @param level - User level as a string given by the client.
   * @param end - End time of the client's session.
   */
  authenticate(t, u, s, c, o, h, C) {
    this.callOnConnection({
      op: "auth",
      mac: t,
      client: u,
      dest: s,
      rand: c,
      t: o,
      level: h,
      end: C
    });
  }
  /**
   * Sends the message to the transport.
   * If not connected, queues the message to send once reconnected.
   */
  callOnConnection(t) {
    this.isConnected ? this.transport?.send(t) : this.once("connection", () => {
      this.transport?.send(t);
    });
  }
  /**
   * Send a set_level request to the server.
   *
   * @param level - Status level (none, error, warning, info).
   * @param [id] - Operation ID to change status level on.
   */
  setStatusLevel(t, u) {
    const s = {
      op: "set_level",
      level: t,
      id: u
    };
    this.callOnConnection(s);
  }
  /**
   * Retrieve a list of action servers in ROS as an array of string.
   *
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getActionServers(t, u = console.error) {
    const s = new re({
      ros: this,
      name: "rosapi/action_servers",
      serviceType: "rosapi/GetActionServers"
    }), c = {};
    s.callService(
      c,
      function(o) {
        t(o.action_servers);
      },
      function(o) {
        u(o);
      }
    );
  }
  /**
   * Retrieve a list of topics in ROS as an array.
   *
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getTopics(t, u = console.error) {
    const s = new re({
      ros: this,
      name: "rosapi/topics",
      serviceType: "rosapi/Topics"
    }), c = {};
    s.callService(
      c,
      function(o) {
        t(o);
      },
      function(o) {
        u(o);
      }
    );
  }
  /**
   * Retrieve a list of topics in ROS as an array of a specific type.
   *
   * @param topicType - The topic type to find.
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getTopicsForType(t, u, s = console.error) {
    const c = new re({
      ros: this,
      name: "rosapi/topics_for_type",
      serviceType: "rosapi/TopicsForType"
    }), o = {
      type: t
    };
    c.callService(
      o,
      function(h) {
        u(h.topics);
      },
      function(h) {
        s(h);
      }
    );
  }
  /**
   * Retrieve a list of active service names in ROS.
   *
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getServices(t, u = console.error) {
    const s = new re({
      ros: this,
      name: "rosapi/services",
      serviceType: "rosapi/Services"
    }), c = {};
    s.callService(
      c,
      function(o) {
        t(o.services);
      },
      function(o) {
        u(o);
      }
    );
  }
  /**
   * Retrieve a list of services in ROS as an array as specific type.
   *
   * @param serviceType - The service type to find.
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getServicesForType(t, u, s = console.error) {
    const c = new re({
      ros: this,
      name: "rosapi/services_for_type",
      serviceType: "rosapi/ServicesForType"
    }), o = {
      type: t
    };
    c.callService(
      o,
      function(h) {
        u(h.services);
      },
      function(h) {
        s(h);
      }
    );
  }
  /**
   * Retrieve the details of a ROS service request.
   *
   * @param type - The type of the service.
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getServiceRequestDetails(t, u, s = console.error) {
    const c = new re({
      ros: this,
      name: "rosapi/service_request_details",
      serviceType: "rosapi/ServiceRequestDetails"
    }), o = {
      type: t
    };
    c.callService(
      o,
      function(h) {
        u(h);
      },
      function(h) {
        s(h);
      }
    );
  }
  /**
   * Retrieve the details of a ROS service response.
   *
   * @param type - The type of the service.
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getServiceResponseDetails(t, u, s = console.error) {
    const c = new re({
      ros: this,
      name: "rosapi/service_response_details",
      serviceType: "rosapi/ServiceResponseDetails"
    }), o = {
      type: t
    };
    c.callService(
      o,
      function(h) {
        u(h);
      },
      function(h) {
        s(h);
      }
    );
  }
  /**
   * Retrieve a list of active node names in ROS.
   *
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getNodes(t, u = console.error) {
    const s = new re({
      ros: this,
      name: "rosapi/nodes",
      serviceType: "rosapi/Nodes"
    }), c = {};
    s.callService(
      c,
      function(o) {
        t(o.nodes);
      },
      function(o) {
        u(o);
      }
    );
  }
  /**
   * Retrieve a list of subscribed topics, publishing topics and services of a specific node.
   *
   * @param node - Name of the node.
   */
  getNodeDetails(t, u, s = console.error) {
    new re({
      ros: this,
      name: "rosapi/node_details",
      serviceType: "rosapi/NodeDetails"
    }).callService({ node: t }, u, s);
  }
  /**
   * Retrieve a list of parameter names from the ROS Parameter Server.
   *
   * @param callback - Function with the following params:
   * @param failedCallback - The callback function when the service call failed with params:
   */
  getParams(t, u = console.error) {
    const s = new re({
      ros: this,
      name: "rosapi/get_param_names",
      serviceType: "rosapi/GetParamNames"
    }), c = {};
    s.callService(
      c,
      function(o) {
        t(o.names);
      },
      function(o) {
        u(o);
      }
    );
  }
  /**
   * Retrieve the type of a ROS topic.
   *
   * @param topic - Name of the topic.
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getTopicType(t, u, s = console.error) {
    const c = new re({
      ros: this,
      name: "rosapi/topic_type",
      serviceType: "rosapi/TopicType"
    }), o = {
      topic: t
    };
    c.callService(
      o,
      function(h) {
        u(h.type);
      },
      function(h) {
        s(h);
      }
    );
  }
  /**
   * Retrieve the type of a ROS service.
   *
   * @param service - Name of the service.
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getServiceType(t, u, s = console.error) {
    const c = new re({
      ros: this,
      name: "rosapi/service_type",
      serviceType: "rosapi/ServiceType"
    }), o = {
      service: t
    };
    c.callService(
      o,
      function(h) {
        u(h.type);
      },
      function(h) {
        s(h);
      }
    );
  }
  /**
   * Retrieve the details of a ROS message.
   *
   * @param message - The name of the message type.
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getMessageDetails(t, u, s = console.error) {
    const c = new re({
      ros: this,
      name: "rosapi/message_details",
      serviceType: "rosapi/MessageDetails"
    }), o = {
      type: t
    };
    c.callService(
      o,
      function(h) {
        u(h.typedefs);
      },
      function(h) {
        s(h);
      }
    );
  }
  /**
   * Decode a typedef array into a dictionary like `rosmsg show foo/bar`.
   *
   * @param defs - Array of type_def dictionary.
   */
  decodeTypeDefs(t) {
    const u = (s, c) => {
      const o = {};
      for (let h = 0; h < s.fieldnames.length; h++) {
        const C = s.fieldarraylen[h], f = s.fieldnames[h], g = s.fieldtypes[h];
        if (f === void 0 || g === void 0)
          throw new Error(
            "Received mismatched type definition vector lengths!"
          );
        if (!g.includes("/"))
          C === -1 ? o[f] = g : o[f] = [g];
        else {
          let D;
          for (const p of c)
            if (p.type === g) {
              D = p;
              break;
            }
          if (D) {
            const p = u(D, c);
            C === -1 ? o[f] = p : o[f] = [p];
          } else
            this.emit("error", `Cannot find ${g} in decodeTypeDefs`);
        }
      }
      return o;
    };
    return t[0] ? u(t[0], t) : {};
  }
  /**
   * @callback getTopicsAndRawTypesCallback
   * @param {Object} result - The result object with the following params:
   * @param {string[]} result.topics - Array of topic names.
   * @param {string[]} result.types - Array of message type names.
   * @param {string[]} result.typedefs_full_text - Array of full definitions of message types, similar to `gendeps --cat`.
   */
  /**
   * @callback getTopicsAndRawTypesFailedCallback
   * @param {string} error - The error message reported by ROS.
   */
  /**
   * Retrieve a list of topics and their associated type definitions.
   *
   * @param callback - Function with the following params:
   * @param [failedCallback] - The callback function when the service call failed with params:
   */
  getTopicsAndRawTypes(t, u = console.error) {
    const s = new re({
      ros: this,
      name: "rosapi/topics_and_raw_types",
      serviceType: "rosapi/TopicsAndRawTypes"
    }), c = {};
    s.callService(
      c,
      function(o) {
        t(o);
      },
      function(o) {
        u(o);
      }
    );
  }
  Topic(t) {
    return new fe({ ros: this, ...t });
  }
  Param(t) {
    return new Kt({ ros: this, ...t });
  }
  Service(t) {
    return new re({ ros: this, ...t });
  }
  TFClient(t) {
    return new tr({ ros: this, ...t });
  }
  ActionClient(t) {
    return new qt({
      ros: this,
      ...t
    });
  }
  SimpleActionServer(t) {
    return new rr({
      ros: this,
      ...t
    });
  }
}
var Be = /* @__PURE__ */ ((n) => (n[n.STATUS_UNKNOWN = 0] = "STATUS_UNKNOWN", n[n.STATUS_ACCEPTED = 1] = "STATUS_ACCEPTED", n[n.STATUS_EXECUTING = 2] = "STATUS_EXECUTING", n[n.STATUS_CANCELING = 3] = "STATUS_CANCELING", n[n.STATUS_SUCCEEDED = 4] = "STATUS_SUCCEEDED", n[n.STATUS_CANCELED = 5] = "STATUS_CANCELED", n[n.STATUS_ABORTED = 6] = "STATUS_ABORTED", n))(Be || {});
class Nt extends Error {
  constructor(t, u) {
    super(`${ir(t)}${u ? `: ${u}` : ""}`), this.name = "GoalError";
  }
}
function ir(n) {
  switch (n) {
    case Be.STATUS_CANCELED:
      return "Action was canceled";
    case Be.STATUS_ABORTED:
      return "Action was aborted";
    case Be.STATUS_CANCELING:
      return "Action is canceling";
    case Be.STATUS_UNKNOWN:
      return "Action status unknown";
    default:
      return `Action failed with status ${String(n)}`;
  }
}
class nr {
  /**
   * @param options
   * @param options.ros - The ROSLIB.Ros connection handle.
   * @param options.name - The action name, like '/fibonacci'.
   * @param options.actionType - The action type, like 'example_interfaces/Fibonacci'.
   */
  constructor({
    ros: t,
    name: u,
    actionType: s
  }) {
    this.isAdvertised = !1, this.#e = null, this.#t = null, this.ros = t, this.name = u, this.actionType = s;
  }
  #e;
  #t;
  /**
   * Sends an action goal. Returns the feedback in the feedback callback while the action is running
   * and the result in the result callback when the action is completed.
   * Does nothing if this action is currently advertised.
   *
   * @param goal - The action goal to send.
   * @param resultCallback - The callback function when the action is completed.
   * @param [feedbackCallback] - The callback function when the action pulishes feedback.
   * @param [failedCallback] - The callback function when the action failed.
   */
  sendGoal(t, u, s, c = console.error) {
    if (this.isAdvertised)
      return;
    const o = `send_action_goal:${this.name}:${nt()}`;
    return this.ros.on(o, (h) => {
      if (Ut(h)) {
        const C = h.status;
        h.result ? C !== Be.STATUS_SUCCEEDED ? c(
          String(new Nt(C, JSON.stringify(h.values)))
        ) : u(h.values) : c(String(new Nt(C, h.values)));
      } else kt(h) && s?.(h.values);
    }), this.ros.callOnConnection({
      op: "send_action_goal",
      id: o,
      action: this.name,
      action_type: this.actionType,
      args: t,
      feedback: !0
    }), o;
  }
  /**
   * Cancels an action goal.
   *
   * @param id - The ID of the action goal to cancel.
   */
  cancelGoal(t) {
    this.ros.callOnConnection({
      op: "cancel_action_goal",
      id: t,
      action: this.name
    });
  }
  /**
   * Advertise the action. This turns the Action object from a client
   * into a server. The callback will be called with every goal sent to this action.
   *
   * @param actionCallback - This works similarly to the callback for a C++ action.
   * @param cancelCallback - A callback function to execute when the action is canceled.
   */
  advertise(t, u) {
    this.isAdvertised || typeof t != "function" || (this.#e = t, this.#t = u, this.ros.on(this.name, (s) => {
      if (Lt(s))
        this.#r(s);
      else
        throw new Error(
          "Received unrelated message on Action server event stream!"
        );
    }), this.ros.callOnConnection({
      op: "advertise_action",
      type: this.actionType,
      action: this.name
    }), this.isAdvertised = !0);
  }
  /**
   * Unadvertise a previously advertised action.
   */
  unadvertise() {
    this.isAdvertised && (this.ros.callOnConnection({
      op: "unadvertise_action",
      action: this.name
    }), this.isAdvertised = !1);
  }
  /**
   * Helper function that executes an action by calling the provided
   * action callback with the auto-generated ID as a user-accessible input.
   * Should not be called manually.
   *
   * @param rosbridgeRequest - The rosbridge request containing the action goal to send and its ID.
   * @param rosbridgeRequest.id - The ID of the action goal.
   * @param rosbridgeRequest.args - The arguments of the action goal.
   */
  #r(t) {
    const u = t.id;
    if (typeof u == "string" && this.ros.on(u, (s) => {
      Pt(s) && this.#t && this.#t(u);
    }), this.#e)
      if (t.args)
        this.#e(t.args, u);
      else
        throw new Error(
          "Received Action goal with no arguments! This should never happen, because rosbridge should fill in blanks!"
        );
  }
  /**
   * Helper function to send action feedback inside an action handler.
   *
   * @param id - The action goal ID.
   * @param feedback - The feedback to send.
   */
  sendFeedback(t, u) {
    this.ros.callOnConnection({
      op: "action_feedback",
      id: t,
      action: this.name,
      values: u
    });
  }
  /**
   * Helper function to set an action as succeeded.
   *
   * @param id - The action goal ID.
   * @param result - The result to set.
   */
  setSucceeded(t, u) {
    this.ros.callOnConnection({
      op: "action_result",
      id: t,
      action: this.name,
      values: u,
      status: Be.STATUS_SUCCEEDED,
      result: !0
    });
  }
  /**
   * Helper function to set an action as canceled.
   *
   * @param id - The action goal ID.
   * @param result - The result to set.
   */
  setCanceled(t, u) {
    this.ros.callOnConnection({
      op: "action_result",
      id: t,
      action: this.name,
      values: u,
      status: Be.STATUS_CANCELED,
      result: !0
    });
  }
  /**
   * Helper function to set an action as failed.
   *
   * @param id - The action goal ID.
   */
  setFailed(t) {
    this.ros.callOnConnection({
      op: "action_result",
      id: t,
      action: this.name,
      status: Be.STATUS_ABORTED,
      result: !1
    });
  }
}
const sr = new TextDecoder();
function ar(n) {
  const t = Uint8Array.from(atob(n), (s) => s.charCodeAt(0)), u = or(t);
  try {
    return JSON.parse(sr.decode(u.data));
  } catch (s) {
    throw new Error("Error parsing PNG JSON contents", { cause: s });
  }
}
function or(n) {
  try {
    return Wt(n);
  } catch (t) {
    throw new Error("Error decoding PNG buffer", { cause: t });
  }
}
class Pr extends ke {
  /**
   * Buffer Map for incoming message fragments.
   */
  #e = /* @__PURE__ */ new Map();
  /**
   * Decodes a raw message received from the transport
   * and emits it as a RosbridgeMessage over the "message" event.
   * If an error occurs, it is emitted as an "error" event.
   *
   * The default implementation handles multiple compression formats
   * and fragment messages. Subclasses may override this method to provide
   * custom handling of raw messages and when to emit messages.
   */
  handleRawMessage(t) {
    try {
      ut(t) ? this.handleRosbridgeMessage(t) : typeof Blob < "u" && t instanceof Blob ? this.handleBsonMessage(t) : t instanceof ArrayBuffer ? this.handleCborMessage(t) : this.handleJsonMessage(String(t));
    } catch (u) {
      this.emit("error", u);
    }
  }
  /**
   * Handles a RosbridgeMessage.
   * If the message is a fragment, it is appended to the fragment buffer.
   * If the message is a PNG, it is decompressed and reprocessed.
   * Otherwise, the message is emitted.
   */
  handleRosbridgeMessage(t) {
    Jt(t) ? this.handleRosbridgeFragmentMessage(t) : Zt(t) ? this.handleRosbridgePngMessage(t) : this.emit("message", t);
  }
  /**
   * Appends a fragment to the current fragment buffer for the message id.
   * If all fragments are received, the message is reconstructed and processed.
   */
  handleRosbridgeFragmentMessage(t) {
    const { id: u, data: s, num: c, total: o } = t;
    if (!u || typeof c != "number" || typeof o != "number" || typeof s != "string")
      return;
    const h = Math.floor(o);
    this.#e.has(u) || this.#e.set(u, {
      fragments: [],
      received: 0,
      total: h
    });
    const C = this.#e.get(u);
    if (!C)
      throw new Error(`Fragment buffer entry missing for id: ${u}`);
    if (c < h && typeof C.fragments[c] > "u" && (C.fragments[c] = s, C.received++), C.received === h) {
      const f = C.fragments.join("");
      let g;
      try {
        g = JSON.parse(f);
      } catch (D) {
        throw new Error("Fragments did not form a valid JSON message!", {
          cause: D
        });
      } finally {
        this.#e.delete(u);
      }
      if (ut(g))
        this.handleRosbridgeMessage(g);
      else
        throw new Error("Received invalid rosbridge message!");
    }
  }
  /**
   * Decompresses a PNG image expecting the result to be a RosbridgeMessage.
   * It is one technique for compressing JSON data.
   */
  handleRosbridgePngMessage(t) {
    const u = ar(t.data);
    if (ut(u))
      this.handleRosbridgeMessage(u);
    else
      throw new Error("Decompressed PNG data was invalid!");
  }
  /**
   * Deserializes a Blob of BSON expecting the result to be a RosbridgeMessage.
   * It is one technique for compressing JSON data.
   */
  handleBsonMessage(t) {
    const u = new FileReader();
    u.onload = () => {
      if (u.result instanceof ArrayBuffer) {
        const s = new Uint8Array(u.result), c = $t(s);
        ut(c) ? this.handleRosbridgeMessage(c) : this.emit("error", new Error("Decoded BSON data was invalid!"));
      }
    }, u.readAsArrayBuffer(t);
  }
  /**
   * Deserializes an ArrayBuffer of CBOR expecting the result to be a RosbridgeMessage.
   * It is one technique for compressing JSON data.
   */
  handleCborMessage(t) {
    const u = jt(new Uint8Array(t));
    if (ut(u))
      this.handleRosbridgeMessage(u);
    else
      throw new Error("Decoded CBOR data was invalid!");
  }
  /**
   * Deserializes a JSON string expecting the result to be a RosbridgeMessage.
   */
  handleJsonMessage(t) {
    const u = JSON.parse(t);
    if (ut(u))
      this.handleRosbridgeMessage(u);
    else
      throw new Error("Received invalid rosbridge message!");
  }
}
class kr extends ke {
  /**
   * @param options
   * @param options.ros - The ROSLIB.Ros connection handle.
   * @param options.serverName - The action server name, like '/fibonacci'.
   * @param options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
   */
  constructor({
    ros: t,
    serverName: u,
    actionName: s
  }) {
    super(), this.ros = t, this.serverName = u, this.actionName = s;
    const c = new fe({
      ros: this.ros,
      name: `${this.serverName}/goal`,
      messageType: `${this.actionName}Goal`
    }), o = new fe({
      ros: this.ros,
      name: `${this.serverName}/feedback`,
      messageType: `${this.actionName}Feedback`
    }), h = new fe({
      ros: this.ros,
      name: `${this.serverName}/status`,
      messageType: "actionlib_msgs/GoalStatusArray"
    }), C = new fe({
      ros: this.ros,
      name: `${this.serverName}/result`,
      messageType: `${this.actionName}Result`
    });
    c.subscribe((f) => {
      this.emit("goal", f);
    }), h.subscribe((f) => {
      f.status_list.forEach((g) => {
        this.emit("status", g);
      });
    }), o.subscribe((f) => {
      this.emit("status", f.status), this.emit("feedback", f.feedback);
    }), C.subscribe((f) => {
      this.emit("status", f.status), this.emit("result", f.result);
    });
  }
}
class pt {
  constructor(t) {
    this.position = new Oe(t?.position), this.orientation = new Qe(t?.orientation);
  }
  /**
   * Apply a transform against this pose.
   *
   * @param tf - The transform to be applied.
   */
  applyTransform(t) {
    this.position.multiplyQuaternion(t.rotation), this.position.add(t.translation);
    const u = new Qe(t.rotation);
    u.multiply(this.orientation), this.orientation = u;
  }
  /**
   * Clone a copy of this pose.
   *
   * @returns The cloned pose.
   */
  clone() {
    return new pt(this);
  }
  /**
   * Multiply this pose with another pose without altering this pose.
   *
   * @returns The result of the multiplication.
   */
  multiply(t) {
    const u = t.clone();
    return u.applyTransform({
      rotation: this.orientation,
      translation: this.position
    }), u;
  }
  /**
   * Compute the inverse of this pose.
   *
   * @returns The inverse of the pose.
   */
  getInverse() {
    const t = this.clone();
    return t.orientation.invert(), t.position.multiplyQuaternion(t.orientation), t.position.x *= -1, t.position.y *= -1, t.position.z *= -1, t;
  }
}
class Ur extends Gt {
  constructor(t) {
    super(t), this.goal_id = "", this.actionClient = new nr({
      ros: this.ros,
      name: this.serverName,
      actionType: "tf2_web_republisher_interfaces/TFSubscription"
    });
  }
  /**
   * Create and send a new goal (or service request) to the tf2_web_republisher
   * based on the current list of TFs.
   */
  updateGoal() {
    const t = {
      source_frames: Object.keys(this.frameInfos),
      target_frame: this.fixedFrame,
      angular_thres: this.angularThres,
      trans_thres: this.transThres,
      rate: this.rate
    };
    this.goal_id !== "" && this.actionClient.cancelGoal(this.goal_id), this.currentGoal = t;
    const u = this.actionClient.sendGoal(
      t,
      () => {
      },
      (s) => {
        this.processTFArray(s);
      }
    );
    typeof u == "string" && (this.goal_id = u), this.republisherUpdateRequested = !1;
  }
  /**
   * Unsubscribe and unadvertise all topics associated with this TFClient.
   */
  dispose() {
    this.goal_id !== "" && this.actionClient.cancelGoal(this.goal_id);
  }
}
var ft = /* @__PURE__ */ ((n) => (n[n.SPHERE = 0] = "SPHERE", n[n.BOX = 1] = "BOX", n[n.CYLINDER = 2] = "CYLINDER", n[n.MESH = 3] = "MESH", n))(ft || {}), H = /* @__PURE__ */ ((n) => (n.Name = "name", n.Type = "type", n.Parent = "parent", n.Link = "link", n.Child = "child", n.Limit = "limit", n.Upper = "upper", n.Lower = "lower", n.Origin = "origin", n.Xyz = "xyz", n.Rpy = "rpy", n.Size = "size", n.Rgba = "rgba", n.Length = "length", n.Radius = "radius", n.Visuals = "visual", n.Texture = "texture", n.Filename = "filename", n.Color = "color", n.Geometry = "geometry", n.Material = "material", n.Scale = "scale", n.Axis = "axis", n))(H || {});
class cr {
  constructor({ xml: t }) {
    this.dimension = null, this.type = ft.BOX;
    const u = t.getAttribute(H.Size)?.split(" ");
    u?.[0] && u[1] && u[2] && (this.dimension = new Oe({
      x: parseFloat(u[0]),
      y: parseFloat(u[1]),
      z: parseFloat(u[2])
    }));
  }
}
class lr {
  constructor({ xml: t }) {
    this.r = 0, this.g = 0, this.b = 0, this.a = 1;
    const u = t.getAttribute(H.Rgba)?.split(" ");
    u?.[0] && u[1] && u[2] && u[3] && (this.r = parseFloat(u[0]), this.g = parseFloat(u[1]), this.b = parseFloat(u[2]), this.a = parseFloat(u[3]));
  }
}
class hr {
  constructor({ xml: t }) {
    this.type = ft.CYLINDER, this.length = parseFloat(t.getAttribute(H.Length) ?? "NaN"), this.radius = parseFloat(t.getAttribute(H.Radius) ?? "NaN");
  }
}
class Vt {
  constructor({ xml: t }) {
    this.textureFilename = null, this.color = null, this.name = t.getAttribute(H.Name) ?? "unknown_name";
    const u = t.getElementsByTagName(H.Texture);
    u[0] && (this.textureFilename = u[0].getAttribute(H.Filename));
    const s = t.getElementsByTagName(H.Color);
    s[0] && (this.color = new lr({
      xml: s[0]
    }));
  }
  isLink() {
    return this.color === null && this.textureFilename === null;
  }
  assign(t) {
    return Object.assign(this, t);
  }
}
class pr {
  constructor({ xml: t }) {
    this.scale = null, this.type = ft.MESH, this.filename = t.getAttribute(H.Filename);
    const u = t.getAttribute(H.Scale)?.split(" ");
    u?.[0] && u[1] && u[2] && (this.scale = new Oe({
      x: parseFloat(u[0]),
      y: parseFloat(u[1]),
      z: parseFloat(u[2])
    }));
  }
}
class fr {
  constructor({ xml: t }) {
    this.radius = NaN, this.type = ft.SPHERE, this.radius = parseFloat(t.getAttribute(H.Radius) ?? "NaN");
  }
}
function zt(n) {
  const t = n.getAttribute(H.Xyz)?.split(" ");
  let u = new Oe();
  t?.[0] && t[1] && t[2] && (u = new Oe({
    x: parseFloat(t[0]),
    y: parseFloat(t[1]),
    z: parseFloat(t[2])
  }));
  const s = n.getAttribute(H.Rpy)?.split(" ");
  let c = new Qe();
  if (s?.[0] && s[1] && s[2]) {
    const o = parseFloat(s[0]), h = parseFloat(s[1]), C = parseFloat(s[2]), f = o / 2, g = h / 2, D = C / 2, p = Math.sin(f) * Math.cos(g) * Math.cos(D) - Math.cos(f) * Math.sin(g) * Math.sin(D), B = Math.cos(f) * Math.sin(g) * Math.cos(D) + Math.sin(f) * Math.cos(g) * Math.sin(D), k = Math.cos(f) * Math.cos(g) * Math.sin(D) - Math.sin(f) * Math.sin(g) * Math.cos(D), Y = Math.cos(f) * Math.cos(g) * Math.cos(D) + Math.sin(f) * Math.sin(g) * Math.sin(D);
    c = new Qe({
      x: p,
      y: B,
      z: k,
      w: Y
    }), c.normalize();
  }
  return new pt({
    position: u,
    orientation: c
  });
}
function Ht(n) {
  return n.nodeType === 1;
}
function dr(n) {
  let t = null;
  for (const s of n.childNodes)
    if (Ht(s)) {
      t = s;
      break;
    }
  if (!t)
    return null;
  const u = {
    xml: t
  };
  switch (t.nodeName) {
    case "sphere":
      return new fr(u);
    case "box":
      return new cr(u);
    case "cylinder":
      return new hr(u);
    case "mesh":
      return new pr(u);
    default:
      return console.warn(`Unknown geometry type ${t.nodeName}`), null;
  }
}
class mr {
  constructor({ xml: t }) {
    this.origin = new pt(), this.geometry = null, this.material = null, this.name = t.getAttribute(H.Name);
    const u = t.getElementsByTagName(H.Origin);
    u[0] && (this.origin = zt(u[0]));
    const s = t.getElementsByTagName(H.Geometry);
    s[0] && (this.geometry = dr(s[0]));
    const c = t.getElementsByTagName(H.Material);
    c[0] && (this.material = new Vt({
      xml: c[0]
    }));
  }
}
class Er {
  constructor({ xml: t }) {
    this.visuals = [], this.name = t.getAttribute(H.Name) ?? "unknown_name";
    const u = t.getElementsByTagName(H.Visuals);
    for (const s of u)
      this.visuals.push(
        new mr({
          xml: s
        })
      );
  }
}
var G = {}, ae = {}, wt;
function st() {
  if (wt) return ae;
  wt = 1;
  function n(M, q, W) {
    if (W === void 0 && (W = Array.prototype), M && typeof W.find == "function")
      return W.find.call(M, q);
    for (var ue = 0; ue < M.length; ue++)
      if (u(M, ue)) {
        var de = M[ue];
        if (q.call(void 0, de, ue, M))
          return de;
      }
  }
  function t(M, q) {
    return q === void 0 && (q = Object), q && typeof q.getOwnPropertyDescriptors == "function" && (M = q.create(null, q.getOwnPropertyDescriptors(M))), q && typeof q.freeze == "function" ? q.freeze(M) : M;
  }
  function u(M, q) {
    return Object.prototype.hasOwnProperty.call(M, q);
  }
  function s(M, q) {
    if (M === null || typeof M != "object")
      throw new TypeError("target is not an object");
    for (var W in q)
      u(q, W) && (M[W] = q[W]);
    return M;
  }
  var c = t({
    allowfullscreen: !0,
    async: !0,
    autofocus: !0,
    autoplay: !0,
    checked: !0,
    controls: !0,
    default: !0,
    defer: !0,
    disabled: !0,
    formnovalidate: !0,
    hidden: !0,
    ismap: !0,
    itemscope: !0,
    loop: !0,
    multiple: !0,
    muted: !0,
    nomodule: !0,
    novalidate: !0,
    open: !0,
    playsinline: !0,
    readonly: !0,
    required: !0,
    reversed: !0,
    selected: !0
  });
  function o(M) {
    return u(c, M.toLowerCase());
  }
  var h = t({
    area: !0,
    base: !0,
    br: !0,
    col: !0,
    embed: !0,
    hr: !0,
    img: !0,
    input: !0,
    link: !0,
    meta: !0,
    param: !0,
    source: !0,
    track: !0,
    wbr: !0
  });
  function C(M) {
    return u(h, M.toLowerCase());
  }
  var f = t({
    script: !1,
    style: !1,
    textarea: !0,
    title: !0
  });
  function g(M) {
    var q = M.toLowerCase();
    return u(f, q) && !f[q];
  }
  function D(M) {
    var q = M.toLowerCase();
    return u(f, q) && f[q];
  }
  function p(M) {
    return M === k.HTML;
  }
  function B(M) {
    return p(M) || M === k.XML_XHTML_APPLICATION;
  }
  var k = t({
    /**
     * `text/html`, the only mime type that triggers treating an XML document as HTML.
     *
     * @see https://www.iana.org/assignments/media-types/text/html IANA MimeType registration
     * @see https://en.wikipedia.org/wiki/HTML Wikipedia
     * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMParser/parseFromString MDN
     * @see https://html.spec.whatwg.org/multipage/dynamic-markup-insertion.html#dom-domparser-parsefromstring
     *      WHATWG HTML Spec
     */
    HTML: "text/html",
    /**
     * `application/xml`, the standard mime type for XML documents.
     *
     * @see https://www.iana.org/assignments/media-types/application/xml IANA MimeType
     *      registration
     * @see https://tools.ietf.org/html/rfc7303#section-9.1 RFC 7303
     * @see https://en.wikipedia.org/wiki/XML_and_MIME Wikipedia
     */
    XML_APPLICATION: "application/xml",
    /**
     * `text/xml`, an alias for `application/xml`.
     *
     * @see https://tools.ietf.org/html/rfc7303#section-9.2 RFC 7303
     * @see https://www.iana.org/assignments/media-types/text/xml IANA MimeType registration
     * @see https://en.wikipedia.org/wiki/XML_and_MIME Wikipedia
     */
    XML_TEXT: "text/xml",
    /**
     * `application/xhtml+xml`, indicates an XML document that has the default HTML namespace,
     * but is parsed as an XML document.
     *
     * @see https://www.iana.org/assignments/media-types/application/xhtml+xml IANA MimeType
     *      registration
     * @see https://dom.spec.whatwg.org/#dom-domimplementation-createdocument WHATWG DOM Spec
     * @see https://en.wikipedia.org/wiki/XHTML Wikipedia
     */
    XML_XHTML_APPLICATION: "application/xhtml+xml",
    /**
     * `image/svg+xml`,
     *
     * @see https://www.iana.org/assignments/media-types/image/svg+xml IANA MimeType registration
     * @see https://www.w3.org/TR/SVG11/ W3C SVG 1.1
     * @see https://en.wikipedia.org/wiki/Scalable_Vector_Graphics Wikipedia
     */
    XML_SVG_IMAGE: "image/svg+xml"
  }), Y = Object.keys(k).map(function(M) {
    return k[M];
  });
  function X(M) {
    return Y.indexOf(M) > -1;
  }
  var te = t({
    /**
     * The XHTML namespace.
     *
     * @see http://www.w3.org/1999/xhtml
     */
    HTML: "http://www.w3.org/1999/xhtml",
    /**
     * The SVG namespace.
     *
     * @see http://www.w3.org/2000/svg
     */
    SVG: "http://www.w3.org/2000/svg",
    /**
     * The `xml:` namespace.
     *
     * @see http://www.w3.org/XML/1998/namespace
     */
    XML: "http://www.w3.org/XML/1998/namespace",
    /**
     * The `xmlns:` namespace.
     *
     * @see https://www.w3.org/2000/xmlns/
     */
    XMLNS: "http://www.w3.org/2000/xmlns/"
  });
  return ae.assign = s, ae.find = n, ae.freeze = t, ae.HTML_BOOLEAN_ATTRIBUTES = c, ae.HTML_RAW_TEXT_ELEMENTS = f, ae.HTML_VOID_ELEMENTS = h, ae.hasDefaultHTMLNamespace = B, ae.hasOwn = u, ae.isHTMLBooleanAttribute = o, ae.isHTMLRawTextElement = g, ae.isHTMLEscapableRawTextElement = D, ae.isHTMLMimeType = p, ae.isHTMLVoidElement = C, ae.isValidMimeType = X, ae.MIME_TYPE = k, ae.NAMESPACE = te, ae;
}
var it = {}, yt;
function mt() {
  if (yt) return it;
  yt = 1;
  var n = st();
  function t(B, k) {
    B.prototype = Object.create(Error.prototype, {
      constructor: { value: B },
      name: { value: B.name, enumerable: !0, writable: k }
    });
  }
  var u = n.freeze({
    /**
     * the default value as defined by the spec
     */
    Error: "Error",
    /**
     * @deprecated
     * Use RangeError instead.
     */
    IndexSizeError: "IndexSizeError",
    /**
     * @deprecated
     * Just to match the related static code, not part of the spec.
     */
    DomstringSizeError: "DomstringSizeError",
    HierarchyRequestError: "HierarchyRequestError",
    WrongDocumentError: "WrongDocumentError",
    InvalidCharacterError: "InvalidCharacterError",
    /**
     * @deprecated
     * Just to match the related static code, not part of the spec.
     */
    NoDataAllowedError: "NoDataAllowedError",
    NoModificationAllowedError: "NoModificationAllowedError",
    NotFoundError: "NotFoundError",
    NotSupportedError: "NotSupportedError",
    InUseAttributeError: "InUseAttributeError",
    InvalidStateError: "InvalidStateError",
    SyntaxError: "SyntaxError",
    InvalidModificationError: "InvalidModificationError",
    NamespaceError: "NamespaceError",
    /**
     * @deprecated
     * Use TypeError for invalid arguments,
     * "NotSupportedError" DOMException for unsupported operations,
     * and "NotAllowedError" DOMException for denied requests instead.
     */
    InvalidAccessError: "InvalidAccessError",
    /**
     * @deprecated
     * Just to match the related static code, not part of the spec.
     */
    ValidationError: "ValidationError",
    /**
     * @deprecated
     * Use TypeError instead.
     */
    TypeMismatchError: "TypeMismatchError",
    SecurityError: "SecurityError",
    NetworkError: "NetworkError",
    AbortError: "AbortError",
    /**
     * @deprecated
     * Just to match the related static code, not part of the spec.
     */
    URLMismatchError: "URLMismatchError",
    QuotaExceededError: "QuotaExceededError",
    TimeoutError: "TimeoutError",
    InvalidNodeTypeError: "InvalidNodeTypeError",
    DataCloneError: "DataCloneError",
    EncodingError: "EncodingError",
    NotReadableError: "NotReadableError",
    UnknownError: "UnknownError",
    ConstraintError: "ConstraintError",
    DataError: "DataError",
    TransactionInactiveError: "TransactionInactiveError",
    ReadOnlyError: "ReadOnlyError",
    VersionError: "VersionError",
    OperationError: "OperationError",
    NotAllowedError: "NotAllowedError",
    OptOutError: "OptOutError"
  }), s = Object.keys(u);
  function c(B) {
    return typeof B == "number" && B >= 1 && B <= 25;
  }
  function o(B) {
    return typeof B == "string" && B.substring(B.length - u.Error.length) === u.Error;
  }
  function h(B, k) {
    c(B) ? (this.name = s[B], this.message = k || "") : (this.message = B, this.name = o(k) ? k : u.Error), Error.captureStackTrace && Error.captureStackTrace(this, h);
  }
  t(h, !0), Object.defineProperties(h.prototype, {
    code: {
      enumerable: !0,
      get: function() {
        var B = s.indexOf(this.name);
        return c(B) ? B : 0;
      }
    }
  });
  for (var C = {
    INDEX_SIZE_ERR: 1,
    DOMSTRING_SIZE_ERR: 2,
    HIERARCHY_REQUEST_ERR: 3,
    WRONG_DOCUMENT_ERR: 4,
    INVALID_CHARACTER_ERR: 5,
    NO_DATA_ALLOWED_ERR: 6,
    NO_MODIFICATION_ALLOWED_ERR: 7,
    NOT_FOUND_ERR: 8,
    NOT_SUPPORTED_ERR: 9,
    INUSE_ATTRIBUTE_ERR: 10,
    INVALID_STATE_ERR: 11,
    SYNTAX_ERR: 12,
    INVALID_MODIFICATION_ERR: 13,
    NAMESPACE_ERR: 14,
    INVALID_ACCESS_ERR: 15,
    VALIDATION_ERR: 16,
    TYPE_MISMATCH_ERR: 17,
    SECURITY_ERR: 18,
    NETWORK_ERR: 19,
    ABORT_ERR: 20,
    URL_MISMATCH_ERR: 21,
    QUOTA_EXCEEDED_ERR: 22,
    TIMEOUT_ERR: 23,
    INVALID_NODE_TYPE_ERR: 24,
    DATA_CLONE_ERR: 25
  }, f = Object.entries(C), g = 0; g < f.length; g++) {
    var D = f[g][0];
    h[D] = f[g][1];
  }
  function p(B, k) {
    this.message = B, this.locator = k, Error.captureStackTrace && Error.captureStackTrace(this, p);
  }
  return t(p), it.DOMException = h, it.DOMExceptionName = u, it.ExceptionCode = C, it.ParseError = p, it;
}
var K = {}, L = {}, _t;
function Yt() {
  if (_t) return L;
  _t = 1;
  function n(ee) {
    try {
      typeof ee != "function" && (ee = RegExp);
      var le = new ee("𝌆", "u").exec("𝌆");
      return !!le && le[0].length === 2;
    } catch {
    }
    return !1;
  }
  var t = n();
  function u(ee) {
    if (ee.source[0] !== "[")
      throw new Error(ee + " can not be used with chars");
    return ee.source.slice(1, ee.source.lastIndexOf("]"));
  }
  function s(ee, le) {
    if (ee.source[0] !== "[")
      throw new Error("/" + ee.source + "/ can not be used with chars_without");
    if (!le || typeof le != "string")
      throw new Error(JSON.stringify(le) + " is not a valid search");
    if (ee.source.indexOf(le) === -1)
      throw new Error('"' + le + '" is not is /' + ee.source + "/");
    if (le === "-" && ee.source.indexOf(le) !== 1)
      throw new Error('"' + le + '" is not at the first postion of /' + ee.source + "/");
    return new RegExp(ee.source.replace(le, ""), t ? "u" : "");
  }
  function c(ee) {
    var le = this;
    return new RegExp(
      Array.prototype.slice.call(arguments).map(function(Me) {
        var xe = typeof Me == "string";
        if (xe && le === void 0 && Me === "|")
          throw new Error("use regg instead of reg to wrap expressions with `|`!");
        return xe ? Me : Me.source;
      }).join(""),
      t ? "mu" : "m"
    );
  }
  function o(ee) {
    if (arguments.length === 0)
      throw new Error("no parameters provided");
    return c.apply(o, ["(?:"].concat(Array.prototype.slice.call(arguments), [")"]));
  }
  var h = "�", C = /[-\x09\x0A\x0D\x20-\x2C\x2E-\uD7FF\uE000-\uFFFD]/;
  t && (C = c("[", u(C), "\\u{10000}-\\u{10FFFF}", "]"));
  var f = /[\x20\x09\x0D\x0A]/, g = u(f), D = c(f, "+"), p = c(f, "*"), B = /[:_a-zA-Z\xC0-\xD6\xD8-\xF6\xF8-\u02FF\u0370-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD]/;
  t && (B = c("[", u(B), "\\u{10000}-\\u{10FFFF}", "]"));
  var k = u(B), Y = c("[", k, u(/[-.0-9\xB7]/), u(/[\u0300-\u036F\u203F-\u2040]/), "]"), X = c(B, Y, "*"), te = c(Y, "+"), M = c("&", X, ";"), q = o(/&#[0-9]+;|&#x[0-9a-fA-F]+;/), W = o(M, "|", q), ue = c("%", X, ";"), de = o(
    c('"', o(/[^%&"]/, "|", ue, "|", W), "*", '"'),
    "|",
    c("'", o(/[^%&']/, "|", ue, "|", W), "*", "'")
  ), m = o('"', o(/[^<&"]/, "|", W), "*", '"', "|", "'", o(/[^<&']/, "|", W), "*", "'"), _ = s(B, ":"), I = s(Y, ":"), U = c(_, I, "*"), $ = c(U, o(":", U), "?"), J = c("^", $, "$"), Ne = c("(", $, ")"), Z = o(/"[^"]*"|'[^']*'/), we = c(/^<\?/, "(", X, ")", o(D, "(", C, "*?)"), "?", /\?>/), l = /[\x20\x0D\x0Aa-zA-Z0-9-'()+,./:=?;!*#@$_%]/, A = o('"', l, '*"', "|", "'", s(l, "'"), "*'"), b = "<!--", v = "-->", y = c(b, o(s(C, "-"), "|", c("-", s(C, "-"))), "*", v), E = "#PCDATA", S = o(
    c(/\(/, p, E, o(p, /\|/, p, $), "*", p, /\)\*/),
    "|",
    c(/\(/, p, E, p, /\)/)
  ), V = /[?*+]?/, O = c(
    /\([^>]+\)/,
    V
    /*regg(choice, '|', seq), _children_quantity*/
  ), T = o("EMPTY", "|", "ANY", "|", S, "|", O), w = "<!ELEMENT", x = c(w, D, o($, "|", ue), D, o(T, "|", ue), p, ">"), P = c("NOTATION", D, /\(/, p, X, o(p, /\|/, p, X), "*", p, /\)/), oe = c(/\(/, p, te, o(p, /\|/, p, te), "*", p, /\)/), Te = o(P, "|", oe), Ee = o(/CDATA|ID|IDREF|IDREFS|ENTITY|ENTITIES|NMTOKEN|NMTOKENS/, "|", Te), ie = o(/#REQUIRED|#IMPLIED/, "|", o(o("#FIXED", D), "?", m)), R = o(D, X, D, Ee, D, ie), Fe = "<!ATTLIST", ye = c(Fe, D, X, R, "*", p, ">"), ce = "about:legacy-compat", Le = o('"' + ce + '"', "|", "'" + ce + "'"), _e = "SYSTEM", Ce = "PUBLIC", Re = o(o(_e, D, Z), "|", o(Ce, D, A, D, Z)), Ue = c(
    "^",
    o(
      o(_e, D, "(?<SystemLiteralOnly>", Z, ")"),
      "|",
      o(Ce, D, "(?<PubidLiteral>", A, ")", D, "(?<SystemLiteral>", Z, ")")
    )
  ), qe = o(D, "NDATA", D, X), Ae = o(de, "|", o(Re, qe, "?")), j = "<!ENTITY", Pe = c(j, D, X, D, Ae, p, ">"), ne = o(de, "|", Re), Ge = c(j, D, "%", D, X, D, ne, p, ">"), at = o(Pe, "|", Ge), Ve = c(Ce, D, A), ze = c("<!NOTATION", D, X, D, o(Re, "|", Ve), p, ">"), F = c(p, "=", p), Q = /1[.]\d+/, De = c(D, "version", F, o("'", Q, "'", "|", '"', Q, '"')), ge = /[A-Za-z][-A-Za-z0-9._]*/, He = o(D, "encoding", F, o('"', ge, '"', "|", "'", ge, "'")), Je = o(D, "standalone", F, o("'", o("yes", "|", "no"), "'", "|", '"', o("yes", "|", "no"), '"')), Ze = c(/^<\?xml/, De, He, "?", Je, "?", p, /\?>/), Ke = "<!DOCTYPE", ot = "<![CDATA[", ct = "]]>", et = /<!\[CDATA\[/, Ye = /\]\]>/, tt = c(C, "*?", Ye), dt = c(et, tt);
  return L.chars = u, L.chars_without = s, L.detectUnicodeSupport = n, L.reg = c, L.regg = o, L.ABOUT_LEGACY_COMPAT = ce, L.ABOUT_LEGACY_COMPAT_SystemLiteral = Le, L.AttlistDecl = ye, L.CDATA_START = ot, L.CDATA_END = ct, L.CDSect = dt, L.Char = C, L.Comment = y, L.COMMENT_START = b, L.COMMENT_END = v, L.DOCTYPE_DECL_START = Ke, L.elementdecl = x, L.EntityDecl = at, L.EntityValue = de, L.ExternalID = Re, L.ExternalID_match = Ue, L.Name = X, L.NotationDecl = ze, L.Reference = W, L.PEReference = ue, L.PI = we, L.PUBLIC = Ce, L.PubidLiteral = A, L.QName = $, L.QName_exact = J, L.QName_group = Ne, L.S = D, L.SChar_s = g, L.S_OPT = p, L.SYSTEM = _e, L.SystemLiteral = Z, L.UNICODE_REPLACEMENT_CHARACTER = h, L.UNICODE_SUPPORT = t, L.XMLDecl = Ze, L;
}
var St;
function Xt() {
  if (St) return K;
  St = 1;
  var n = st(), t = n.find, u = n.hasDefaultHTMLNamespace, s = n.hasOwn, c = n.isHTMLMimeType, o = n.isHTMLRawTextElement, h = n.isHTMLVoidElement, C = n.MIME_TYPE, f = n.NAMESPACE, g = Symbol(), D = mt(), p = D.DOMException, B = D.DOMExceptionName, k = Yt();
  function Y(e) {
    if (e !== g)
      throw new TypeError("Illegal constructor");
  }
  function X(e) {
    return e !== "";
  }
  function te(e) {
    return e ? e.split(/[\t\n\f\r ]+/).filter(X) : [];
  }
  function M(e, r) {
    return s(e, r) || (e[r] = !0), e;
  }
  function q(e) {
    if (!e) return [];
    var r = te(e);
    return Object.keys(r.reduce(M, {}));
  }
  function W(e) {
    return function(r) {
      return e && e.indexOf(r) !== -1;
    };
  }
  function ue(e) {
    if (!k.QName_exact.test(e))
      throw new p(p.INVALID_CHARACTER_ERR, 'invalid character in qualified name "' + e + '"');
  }
  function de(e, r) {
    ue(r), e = e || null;
    var i = null, a = r;
    if (r.indexOf(":") >= 0) {
      var d = r.split(":");
      i = d[0], a = d[1];
    }
    if (i !== null && e === null)
      throw new p(p.NAMESPACE_ERR, "prefix is non-null and namespace is null");
    if (i === "xml" && e !== n.NAMESPACE.XML)
      throw new p(p.NAMESPACE_ERR, 'prefix is "xml" and namespace is not the XML namespace');
    if ((i === "xmlns" || r === "xmlns") && e !== n.NAMESPACE.XMLNS)
      throw new p(
        p.NAMESPACE_ERR,
        'either qualifiedName or prefix is "xmlns" and namespace is not the XMLNS namespace'
      );
    if (e === n.NAMESPACE.XMLNS && i !== "xmlns" && r !== "xmlns")
      throw new p(
        p.NAMESPACE_ERR,
        'namespace is the XMLNS namespace and neither qualifiedName nor prefix is "xmlns"'
      );
    return [e, i, a];
  }
  function m(e, r) {
    for (var i in e)
      s(e, i) && (r[i] = e[i]);
  }
  function _(e, r) {
    var i = e.prototype;
    if (!(i instanceof r)) {
      let a = function() {
      };
      a.prototype = r.prototype, a = new a(), m(i, a), e.prototype = i = a;
    }
    i.constructor != e && (typeof e != "function" && console.error("unknown Class:" + e), i.constructor = e);
  }
  var I = {}, U = I.ELEMENT_NODE = 1, $ = I.ATTRIBUTE_NODE = 2, J = I.TEXT_NODE = 3, Ne = I.CDATA_SECTION_NODE = 4, Z = I.ENTITY_REFERENCE_NODE = 5, we = I.ENTITY_NODE = 6, l = I.PROCESSING_INSTRUCTION_NODE = 7, A = I.COMMENT_NODE = 8, b = I.DOCUMENT_NODE = 9, v = I.DOCUMENT_TYPE_NODE = 10, y = I.DOCUMENT_FRAGMENT_NODE = 11, E = I.NOTATION_NODE = 12, S = n.freeze({
    DOCUMENT_POSITION_DISCONNECTED: 1,
    DOCUMENT_POSITION_PRECEDING: 2,
    DOCUMENT_POSITION_FOLLOWING: 4,
    DOCUMENT_POSITION_CONTAINS: 8,
    DOCUMENT_POSITION_CONTAINED_BY: 16,
    DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC: 32
  });
  function V(e, r) {
    if (r.length < e.length) return V(r, e);
    var i = null;
    for (var a in e) {
      if (e[a] !== r[a]) return i;
      i = e[a];
    }
    return i;
  }
  function O(e) {
    return e.guid || (e.guid = Math.random()), e.guid;
  }
  function T() {
  }
  T.prototype = {
    /**
     * The number of nodes in the list. The range of valid child node indices is 0 to length-1
     * inclusive.
     *
     * @type {number}
     */
    length: 0,
    /**
     * Returns the item at `index`. If index is greater than or equal to the number of nodes in
     * the list, this returns null.
     *
     * @param index
     * Unsigned long Index into the collection.
     * @returns {Node | null}
     * The node at position `index` in the NodeList,
     * or null if that is not a valid index.
     */
    item: function(e) {
      return e >= 0 && e < this.length ? this[e] : null;
    },
    /**
     * Returns a string representation of the NodeList.
     *
     * @param {unknown} nodeFilter
     * __A filter function? Not implemented according to the spec?__.
     * @returns {string}
     * A string representation of the NodeList.
     */
    toString: function(e) {
      for (var r = [], i = 0; i < this.length; i++)
        xe(this[i], r, e);
      return r.join("");
    },
    /**
     * Filters the NodeList based on a predicate.
     *
     * @param {function(Node): boolean} predicate
     * - A predicate function to filter the NodeList.
     * @returns {Node[]}
     * An array of nodes that satisfy the predicate.
     * @private
     */
    filter: function(e) {
      return Array.prototype.filter.call(this, e);
    },
    /**
     * Returns the first index at which a given node can be found in the NodeList, or -1 if it is
     * not present.
     *
     * @param {Node} item
     * - The Node item to locate in the NodeList.
     * @returns {number}
     * The first index of the node in the NodeList; -1 if not found.
     * @private
     */
    indexOf: function(e) {
      return Array.prototype.indexOf.call(this, e);
    }
  }, T.prototype[Symbol.iterator] = function() {
    var e = this, r = 0;
    return {
      next: function() {
        return r < e.length ? {
          value: e[r++],
          done: !1
        } : {
          done: !0
        };
      },
      return: function() {
        return {
          done: !0
        };
      }
    };
  };
  function w(e, r) {
    this._node = e, this._refresh = r, x(this);
  }
  function x(e) {
    var r = e._node._inc || e._node.ownerDocument._inc;
    if (e._inc !== r) {
      var i = e._refresh(e._node);
      if (Ct(e, "length", i.length), !e.$$length || i.length < e.$$length)
        for (var a = i.length; a in e; a++)
          s(e, a) && delete e[a];
      m(i, e), e._inc = r;
    }
  }
  w.prototype.item = function(e) {
    return x(this), this[e] || null;
  }, _(w, T);
  function P() {
  }
  function oe(e, r) {
    for (var i = 0; i < e.length; ) {
      if (e[i] === r)
        return i;
      i++;
    }
  }
  function Te(e, r, i, a) {
    if (a ? r[oe(r, a)] = i : (r[r.length] = i, r.length++), e) {
      i.ownerElement = e;
      var d = e.ownerDocument;
      d && (a && _e(d, e, a), Le(d, e, i));
    }
  }
  function Ee(e, r, i) {
    var a = oe(r, i);
    if (a >= 0) {
      for (var d = r.length - 1; a <= d; )
        r[a] = r[++a];
      if (r.length = d, e) {
        var N = e.ownerDocument;
        N && _e(N, e, i), i.ownerElement = null;
      }
    }
  }
  P.prototype = {
    length: 0,
    item: T.prototype.item,
    /**
     * Get an attribute by name. Note: Name is in lower case in case of HTML namespace and
     * document.
     *
     * @param {string} localName
     * The local name of the attribute.
     * @returns {Attr | null}
     * The attribute with the given local name, or null if no such attribute exists.
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-get-by-name
     */
    getNamedItem: function(e) {
      this._ownerElement && this._ownerElement._isInHTMLDocumentAndNamespace() && (e = e.toLowerCase());
      for (var r = 0; r < this.length; ) {
        var i = this[r];
        if (i.nodeName === e)
          return i;
        r++;
      }
      return null;
    },
    /**
     * Set an attribute.
     *
     * @param {Attr} attr
     * The attribute to set.
     * @returns {Attr | null}
     * The old attribute with the same local name and namespace URI as the new one, or null if no
     * such attribute exists.
     * @throws {DOMException}
     * With code:
     * - {@link INUSE_ATTRIBUTE_ERR} - If the attribute is already an attribute of another
     * element.
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-set
     */
    setNamedItem: function(e) {
      var r = e.ownerElement;
      if (r && r !== this._ownerElement)
        throw new p(p.INUSE_ATTRIBUTE_ERR);
      var i = this.getNamedItemNS(e.namespaceURI, e.localName);
      return i === e ? e : (Te(this._ownerElement, this, e, i), i);
    },
    /**
     * Set an attribute, replacing an existing attribute with the same local name and namespace
     * URI if one exists.
     *
     * @param {Attr} attr
     * The attribute to set.
     * @returns {Attr | null}
     * The old attribute with the same local name and namespace URI as the new one, or null if no
     * such attribute exists.
     * @throws {DOMException}
     * Throws a DOMException with the name "InUseAttributeError" if the attribute is already an
     * attribute of another element.
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-set
     */
    setNamedItemNS: function(e) {
      return this.setNamedItem(e);
    },
    /**
     * Removes an attribute specified by the local name.
     *
     * @param {string} localName
     * The local name of the attribute to be removed.
     * @returns {Attr}
     * The attribute node that was removed.
     * @throws {DOMException}
     * With code:
     * - {@link DOMException.NOT_FOUND_ERR} if no attribute with the given name is found.
     * @see https://dom.spec.whatwg.org/#dom-namednodemap-removenameditem
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-remove-by-name
     */
    removeNamedItem: function(e) {
      var r = this.getNamedItem(e);
      if (!r)
        throw new p(p.NOT_FOUND_ERR, e);
      return Ee(this._ownerElement, this, r), r;
    },
    /**
     * Removes an attribute specified by the namespace and local name.
     *
     * @param {string | null} namespaceURI
     * The namespace URI of the attribute to be removed.
     * @param {string} localName
     * The local name of the attribute to be removed.
     * @returns {Attr}
     * The attribute node that was removed.
     * @throws {DOMException}
     * With code:
     * - {@link DOMException.NOT_FOUND_ERR} if no attribute with the given namespace URI and local
     * name is found.
     * @see https://dom.spec.whatwg.org/#dom-namednodemap-removenameditemns
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-remove-by-namespace
     */
    removeNamedItemNS: function(e, r) {
      var i = this.getNamedItemNS(e, r);
      if (!i)
        throw new p(p.NOT_FOUND_ERR, e ? e + " : " + r : r);
      return Ee(this._ownerElement, this, i), i;
    },
    /**
     * Get an attribute by namespace and local name.
     *
     * @param {string | null} namespaceURI
     * The namespace URI of the attribute.
     * @param {string} localName
     * The local name of the attribute.
     * @returns {Attr | null}
     * The attribute with the given namespace URI and local name, or null if no such attribute
     * exists.
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-get-by-namespace
     */
    getNamedItemNS: function(e, r) {
      e || (e = null);
      for (var i = 0; i < this.length; ) {
        var a = this[i];
        if (a.localName === r && a.namespaceURI === e)
          return a;
        i++;
      }
      return null;
    }
  }, P.prototype[Symbol.iterator] = function() {
    var e = this, r = 0;
    return {
      next: function() {
        return r < e.length ? {
          value: e[r++],
          done: !1
        } : {
          done: !0
        };
      },
      return: function() {
        return {
          done: !0
        };
      }
    };
  };
  function ie() {
  }
  ie.prototype = {
    /**
     * Test if the DOM implementation implements a specific feature and version, as specified in
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/core.html#DOMFeatures DOM Features}.
     *
     * The DOMImplementation.hasFeature() method returns a Boolean flag indicating if a given
     * feature is supported. The different implementations fairly diverged in what kind of
     * features were reported. The latest version of the spec settled to force this method to
     * always return true, where the functionality was accurate and in use.
     *
     * @deprecated
     * It is deprecated and modern browsers return true in all cases.
     * @function DOMImplementation#hasFeature
     * @param {string} feature
     * The name of the feature to test.
     * @param {string} [version]
     * This is the version number of the feature to test.
     * @returns {boolean}
     * Always returns true.
     * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMImplementation/hasFeature MDN
     * @see https://www.w3.org/TR/REC-DOM-Level-1/level-one-core.html#ID-5CED94D7 DOM Level 1 Core
     * @see https://dom.spec.whatwg.org/#dom-domimplementation-hasfeature DOM Living Standard
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#ID-5CED94D7 DOM Level 3 Core
     */
    hasFeature: function(e, r) {
      return !0;
    },
    /**
     * Creates a DOM Document object of the specified type with its document element. Note that
     * based on the {@link DocumentType}
     * given to create the document, the implementation may instantiate specialized
     * {@link Document} objects that support additional features than the "Core", such as "HTML"
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#DOM2HTML DOM Level 2 HTML}.
     * On the other hand, setting the {@link DocumentType} after the document was created makes
     * this very unlikely to happen. Alternatively, specialized {@link Document} creation methods,
     * such as createHTMLDocument
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#DOM2HTML DOM Level 2 HTML},
     * can be used to obtain specific types of {@link Document} objects.
     *
     * __It behaves slightly different from the description in the living standard__:
     * - There is no interface/class `XMLDocument`, it returns a `Document`
     * instance (with it's `type` set to `'xml'`).
     * - `encoding`, `mode`, `origin`, `url` fields are currently not declared.
     *
     * @function DOMImplementation.createDocument
     * @param {string | null} namespaceURI
     * The
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/glossary.html#dt-namespaceURI namespace URI}
     * of the document element to create or null.
     * @param {string | null} qualifiedName
     * The
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/glossary.html#dt-qualifiedname qualified name}
     * of the document element to be created or null.
     * @param {DocumentType | null} [doctype=null]
     * The type of document to be created or null. When doctype is not null, its
     * {@link Node#ownerDocument} attribute is set to the document being created. Default is
     * `null`
     * @returns {Document}
     * A new {@link Document} object with its document element. If the NamespaceURI,
     * qualifiedName, and doctype are null, the returned {@link Document} is empty with no
     * document element.
     * @throws {DOMException}
     * With code:
     *
     * - `INVALID_CHARACTER_ERR`: Raised if the specified qualified name is not an XML name
     * according to {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#XML XML 1.0}.
     * - `NAMESPACE_ERR`: Raised if the qualifiedName is malformed, if the qualifiedName has a
     * prefix and the namespaceURI is null, or if the qualifiedName is null and the namespaceURI
     * is different from null, or if the qualifiedName has a prefix that is "xml" and the
     * namespaceURI is different from "{@link http://www.w3.org/XML/1998/namespace}"
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#Namespaces XML Namespaces},
     * or if the DOM implementation does not support the "XML" feature but a non-null namespace
     * URI was provided, since namespaces were defined by XML.
     * - `WRONG_DOCUMENT_ERR`: Raised if doctype has already been used with a different document
     * or was created from a different implementation.
     * - `NOT_SUPPORTED_ERR`: May be raised if the implementation does not support the feature
     * "XML" and the language exposed through the Document does not support XML Namespaces (such
     * as {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#HTML40 HTML 4.01}).
     * @since DOM Level 2.
     * @see {@link #createHTMLDocument}
     * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMImplementation/createDocument MDN
     * @see https://dom.spec.whatwg.org/#dom-domimplementation-createdocument DOM Living Standard
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Level-2-Core-DOM-createDocument DOM
     *      Level 3 Core
     * @see https://www.w3.org/TR/DOM-Level-2-Core/core.html#Level-2-Core-DOM-createDocument DOM
     *      Level 2 Core (initial)
     */
    createDocument: function(e, r, i) {
      var a = C.XML_APPLICATION;
      e === f.HTML ? a = C.XML_XHTML_APPLICATION : e === f.SVG && (a = C.XML_SVG_IMAGE);
      var d = new ce(g, { contentType: a });
      if (d.implementation = this, d.childNodes = new T(), d.doctype = i || null, i && d.appendChild(i), r) {
        var N = d.createElementNS(e, r);
        d.appendChild(N);
      }
      return d;
    },
    /**
     * Creates an empty DocumentType node. Entity declarations and notations are not made
     * available. Entity reference expansions and default attribute additions do not occur.
     *
     * **This behavior is slightly different from the one in the specs**:
     * - `encoding`, `mode`, `origin`, `url` fields are currently not declared.
     * - `publicId` and `systemId` contain the raw data including any possible quotes,
     *   so they can always be serialized back to the original value
     * - `internalSubset` contains the raw string between `[` and `]` if present,
     *   but is not parsed or validated in any form.
     *
     * @function DOMImplementation#createDocumentType
     * @param {string} qualifiedName
     * The {@link https://www.w3.org/TR/DOM-Level-3-Core/glossary.html#dt-qualifiedname qualified
     * name} of the document type to be created.
     * @param {string} [publicId]
     * The external subset public identifier.
     * @param {string} [systemId]
     * The external subset system identifier.
     * @param {string} [internalSubset]
     * the internal subset or an empty string if it is not present
     * @returns {DocumentType}
     * A new {@link DocumentType} node with {@link Node#ownerDocument} set to null.
     * @throws {DOMException}
     * With code:
     *
     * - `INVALID_CHARACTER_ERR`: Raised if the specified qualified name is not an XML name
     * according to {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#XML XML 1.0}.
     * - `NAMESPACE_ERR`: Raised if the qualifiedName is malformed.
     * - `NOT_SUPPORTED_ERR`: May be raised if the implementation does not support the feature
     * "XML" and the language exposed through the Document does not support XML Namespaces (such
     * as {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#HTML40 HTML 4.01}).
     * @since DOM Level 2.
     * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMImplementation/createDocumentType
     *      MDN
     * @see https://dom.spec.whatwg.org/#dom-domimplementation-createdocumenttype DOM Living
     *      Standard
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Level-3-Core-DOM-createDocType DOM
     *      Level 3 Core
     * @see https://www.w3.org/TR/DOM-Level-2-Core/core.html#Level-2-Core-DOM-createDocType DOM
     *      Level 2 Core
     * @see https://github.com/xmldom/xmldom/blob/master/CHANGELOG.md#050
     * @see https://www.w3.org/TR/DOM-Level-2-Core/#core-ID-Core-DocType-internalSubset
     * @prettierignore
     */
    createDocumentType: function(e, r, i, a) {
      ue(e);
      var d = new Ke(g);
      return d.name = e, d.nodeName = e, d.publicId = r || "", d.systemId = i || "", d.internalSubset = a || "", d.childNodes = new T(), d;
    },
    /**
     * Returns an HTML document, that might already have a basic DOM structure.
     *
     * __It behaves slightly different from the description in the living standard__:
     * - If the first argument is `false` no initial nodes are added (steps 3-7 in the specs are
     * omitted)
     * - `encoding`, `mode`, `origin`, `url` fields are currently not declared.
     *
     * @param {string | false} [title]
     * A string containing the title to give the new HTML document.
     * @returns {Document}
     * The HTML document.
     * @since WHATWG Living Standard.
     * @see {@link #createDocument}
     * @see https://dom.spec.whatwg.org/#dom-domimplementation-createhtmldocument
     * @see https://dom.spec.whatwg.org/#html-document
     */
    createHTMLDocument: function(e) {
      var r = new ce(g, { contentType: C.HTML });
      if (r.implementation = this, r.childNodes = new T(), e !== !1) {
        r.doctype = this.createDocumentType("html"), r.doctype.ownerDocument = r, r.appendChild(r.doctype);
        var i = r.createElement("html");
        r.appendChild(i);
        var a = r.createElement("head");
        if (i.appendChild(a), typeof e == "string") {
          var d = r.createElement("title");
          d.appendChild(r.createTextNode(e)), a.appendChild(d);
        }
        i.appendChild(r.createElement("body"));
      }
      return r;
    }
  };
  function R(e) {
    Y(e);
  }
  R.prototype = {
    /**
     * The first child of this node.
     *
     * @type {Node | null}
     */
    firstChild: null,
    /**
     * The last child of this node.
     *
     * @type {Node | null}
     */
    lastChild: null,
    /**
     * The previous sibling of this node.
     *
     * @type {Node | null}
     */
    previousSibling: null,
    /**
     * The next sibling of this node.
     *
     * @type {Node | null}
     */
    nextSibling: null,
    /**
     * The parent node of this node.
     *
     * @type {Node | null}
     */
    parentNode: null,
    /**
     * The parent element of this node.
     *
     * @type {Element | null}
     */
    get parentElement() {
      return this.parentNode && this.parentNode.nodeType === this.ELEMENT_NODE ? this.parentNode : null;
    },
    /**
     * The child nodes of this node.
     *
     * @type {NodeList}
     */
    childNodes: null,
    /**
     * The document object associated with this node.
     *
     * @type {Document | null}
     */
    ownerDocument: null,
    /**
     * The value of this node.
     *
     * @type {string | null}
     */
    nodeValue: null,
    /**
     * The namespace URI of this node.
     *
     * @type {string | null}
     */
    namespaceURI: null,
    /**
     * The prefix of the namespace for this node.
     *
     * @type {string | null}
     */
    prefix: null,
    /**
     * The local part of the qualified name of this node.
     *
     * @type {string | null}
     */
    localName: null,
    /**
     * The baseURI is currently always `about:blank`,
     * since that's what happens when you create a document from scratch.
     *
     * @type {'about:blank'}
     */
    baseURI: "about:blank",
    /**
     * Is true if this node is part of a document.
     *
     * @type {boolean}
     */
    get isConnected() {
      var e = this.getRootNode();
      return e && e.nodeType === e.DOCUMENT_NODE;
    },
    /**
     * Checks whether `other` is an inclusive descendant of this node.
     *
     * @param {Node | null | undefined} other
     * The node to check.
     * @returns {boolean}
     * True if `other` is an inclusive descendant of this node; false otherwise.
     * @see https://dom.spec.whatwg.org/#dom-node-contains
     */
    contains: function(e) {
      if (!e) return !1;
      var r = e;
      do {
        if (this === r) return !0;
        r = e.parentNode;
      } while (r);
      return !1;
    },
    /**
     * @typedef GetRootNodeOptions
     * @property {boolean} [composed=false]
     */
    /**
     * Searches for the root node of this node.
     *
     * **This behavior is slightly different from the in the specs**:
     * - ignores `options.composed`, since `ShadowRoot`s are unsupported, always returns root.
     *
     * @param {GetRootNodeOptions} [options]
     * @returns {Node}
     * Root node.
     * @see https://dom.spec.whatwg.org/#dom-node-getrootnode
     * @see https://dom.spec.whatwg.org/#concept-shadow-including-root
     */
    getRootNode: function(e) {
      var r = this;
      do {
        if (!r.parentNode)
          return r;
        r = r.parentNode;
      } while (r);
    },
    /**
     * Checks whether the given node is equal to this node.
     *
     * @param {Node} [otherNode]
     * @see https://dom.spec.whatwg.org/#concept-node-equals
     */
    isEqualNode: function(e) {
      if (!e || this.nodeType !== e.nodeType) return !1;
      switch (this.nodeType) {
        case this.DOCUMENT_TYPE_NODE:
          if (this.name !== e.name || this.publicId !== e.publicId || this.systemId !== e.systemId) return !1;
          break;
        case this.ELEMENT_NODE:
          if (this.namespaceURI !== e.namespaceURI || this.prefix !== e.prefix || this.localName !== e.localName || this.attributes.length !== e.attributes.length) return !1;
          for (var r = 0; r < this.attributes.length; r++) {
            var i = this.attributes.item(r);
            if (!i.isEqualNode(e.getAttributeNodeNS(i.namespaceURI, i.localName)))
              return !1;
          }
          break;
        case this.ATTRIBUTE_NODE:
          if (this.namespaceURI !== e.namespaceURI || this.localName !== e.localName || this.value !== e.value) return !1;
          break;
        case this.PROCESSING_INSTRUCTION_NODE:
          if (this.target !== e.target || this.data !== e.data)
            return !1;
          break;
        case this.TEXT_NODE:
        case this.COMMENT_NODE:
          if (this.data !== e.data) return !1;
          break;
      }
      if (this.childNodes.length !== e.childNodes.length)
        return !1;
      for (var r = 0; r < this.childNodes.length; r++)
        if (!this.childNodes[r].isEqualNode(e.childNodes[r]))
          return !1;
      return !0;
    },
    /**
     * Checks whether or not the given node is this node.
     *
     * @param {Node} [otherNode]
     */
    isSameNode: function(e) {
      return this === e;
    },
    /**
     * Inserts a node before a reference node as a child of this node.
     *
     * @param {Node} newChild
     * The new child node to be inserted.
     * @param {Node | null} refChild
     * The reference node before which newChild will be inserted.
     * @returns {Node}
     * The new child node successfully inserted.
     * @throws {DOMException}
     * Throws a DOMException if inserting the node would result in a DOM tree that is not
     * well-formed, or if `child` is provided but is not a child of `parent`.
     * See {@link _insertBefore} for more details.
     * @since Modified in DOM L2
     */
    insertBefore: function(e, r) {
      return F(this, e, r);
    },
    /**
     * Replaces an old child node with a new child node within this node.
     *
     * @param {Node} newChild
     * The new node that is to replace the old node.
     * If it already exists in the DOM, it is removed from its original position.
     * @param {Node} oldChild
     * The existing child node to be replaced.
     * @returns {Node}
     * Returns the replaced child node.
     * @throws {DOMException}
     * Throws a DOMException if replacing the node would result in a DOM tree that is not
     * well-formed, or if `oldChild` is not a child of `this`.
     * This can also occur if the pre-replacement validity assertion fails.
     * See {@link _insertBefore}, {@link Node.removeChild}, and
     * {@link assertPreReplacementValidityInDocument} for more details.
     * @see https://dom.spec.whatwg.org/#concept-node-replace
     */
    replaceChild: function(e, r) {
      F(this, e, r, ze), r && this.removeChild(r);
    },
    /**
     * Removes an existing child node from this node.
     *
     * @param {Node} oldChild
     * The child node to be removed.
     * @returns {Node}
     * Returns the removed child node.
     * @throws {DOMException}
     * Throws a DOMException if `oldChild` is not a child of `this`.
     * See {@link _removeChild} for more details.
     */
    removeChild: function(e) {
      return Re(this, e);
    },
    /**
     * Appends a child node to this node.
     *
     * @param {Node} newChild
     * The child node to be appended to this node.
     * If it already exists in the DOM, it is removed from its original position.
     * @returns {Node}
     * Returns the appended child node.
     * @throws {DOMException}
     * Throws a DOMException if appending the node would result in a DOM tree that is not
     * well-formed, or if `newChild` is not a valid Node.
     * See {@link insertBefore} for more details.
     */
    appendChild: function(e) {
      return this.insertBefore(e, null);
    },
    /**
     * Determines whether this node has any child nodes.
     *
     * @returns {boolean}
     * Returns true if this node has any child nodes, and false otherwise.
     */
    hasChildNodes: function() {
      return this.firstChild != null;
    },
    /**
     * Creates a copy of the calling node.
     *
     * @param {boolean} deep
     * If true, the contents of the node are recursively copied.
     * If false, only the node itself (and its attributes, if it is an element) are copied.
     * @returns {Node}
     * Returns the newly created copy of the node.
     * @throws {DOMException}
     * May throw a DOMException if operations within {@link Element#setAttributeNode} or
     * {@link Node#appendChild} (which are potentially invoked in this method) do not meet their
     * specific constraints.
     * @see {@link cloneNode}
     */
    cloneNode: function(e) {
      return Et(this.ownerDocument || this, this, e);
    },
    /**
     * Puts the specified node and all of its subtree into a "normalized" form. In a normalized
     * subtree, no text nodes in the subtree are empty and there are no adjacent text nodes.
     *
     * Specifically, this method merges any adjacent text nodes (i.e., nodes for which `nodeType`
     * is `TEXT_NODE`) into a single node with the combined data. It also removes any empty text
     * nodes.
     *
     * This method operates recursively, so it also normalizes any and all descendent nodes within
     * the subtree.
     *
     * @throws {DOMException}
     * May throw a DOMException if operations within removeChild or appendData (which are
     * potentially invoked in this method) do not meet their specific constraints.
     * @since Modified in DOM Level 2
     * @see {@link Node.removeChild}
     * @see {@link CharacterData.appendData}
     */
    normalize: function() {
      for (var e = this.firstChild; e; ) {
        var r = e.nextSibling;
        r && r.nodeType == J && e.nodeType == J ? (this.removeChild(r), e.appendData(r.data)) : (e.normalize(), e = r);
      }
    },
    /**
     * Checks whether the DOM implementation implements a specific feature and its version.
     *
     * @deprecated
     * Since `DOMImplementation.hasFeature` is deprecated and always returns true.
     * @param {string} feature
     * The package name of the feature to test. This is the same name that can be passed to the
     * method `hasFeature` on `DOMImplementation`.
     * @param {string} version
     * This is the version number of the package name to test.
     * @returns {boolean}
     * Returns true in all cases in the current implementation.
     * @since Introduced in DOM Level 2
     * @see {@link DOMImplementation.hasFeature}
     */
    isSupported: function(e, r) {
      return this.ownerDocument.implementation.hasFeature(e, r);
    },
    /**
     * Look up the prefix associated to the given namespace URI, starting from this node.
     * **The default namespace declarations are ignored by this method.**
     * See Namespace Prefix Lookup for details on the algorithm used by this method.
     *
     * **This behavior is different from the in the specs**:
     * - no node type specific handling
     * - uses the internal attribute _nsMap for resolving namespaces that is updated when changing attributes
     *
     * @param {string | null} namespaceURI
     * The namespace URI for which to find the associated prefix.
     * @returns {string | null}
     * The associated prefix, if found; otherwise, null.
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Node3-lookupNamespacePrefix
     * @see https://www.w3.org/TR/DOM-Level-3-Core/namespaces-algorithms.html#lookupNamespacePrefixAlgo
     * @see https://dom.spec.whatwg.org/#dom-node-lookupprefix
     * @see https://github.com/xmldom/xmldom/issues/322
     * @prettierignore
     */
    lookupPrefix: function(e) {
      for (var r = this; r; ) {
        var i = r._nsMap;
        if (i) {
          for (var a in i)
            if (s(i, a) && i[a] === e)
              return a;
        }
        r = r.nodeType == $ ? r.ownerDocument : r.parentNode;
      }
      return null;
    },
    /**
     * This function is used to look up the namespace URI associated with the given prefix,
     * starting from this node.
     *
     * **This behavior is different from the in the specs**:
     * - no node type specific handling
     * - uses the internal attribute _nsMap for resolving namespaces that is updated when changing attributes
     *
     * @param {string | null} prefix
     * The prefix for which to find the associated namespace URI.
     * @returns {string | null}
     * The associated namespace URI, if found; otherwise, null.
     * @since DOM Level 3
     * @see https://dom.spec.whatwg.org/#dom-node-lookupnamespaceuri
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Node3-lookupNamespaceURI
     * @prettierignore
     */
    lookupNamespaceURI: function(e) {
      for (var r = this; r; ) {
        var i = r._nsMap;
        if (i && s(i, e))
          return i[e];
        r = r.nodeType == $ ? r.ownerDocument : r.parentNode;
      }
      return null;
    },
    /**
     * Determines whether the given namespace URI is the default namespace.
     *
     * The function works by looking up the prefix associated with the given namespace URI. If no
     * prefix is found (i.e., the namespace URI is not registered in the namespace map of this
     * node or any of its ancestors), it returns `true`, implying the namespace URI is considered
     * the default.
     *
     * **This behavior is different from the in the specs**:
     * - no node type specific handling
     * - uses the internal attribute _nsMap for resolving namespaces that is updated when changing attributes
     *
     * @param {string | null} namespaceURI
     * The namespace URI to be checked.
     * @returns {boolean}
     * Returns true if the given namespace URI is the default namespace, false otherwise.
     * @since DOM Level 3
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Node3-isDefaultNamespace
     * @see https://dom.spec.whatwg.org/#dom-node-isdefaultnamespace
     * @prettierignore
     */
    isDefaultNamespace: function(e) {
      var r = this.lookupPrefix(e);
      return r == null;
    },
    /**
     * Compares the reference node with a node with regard to their position in the document and
     * according to the document order.
     *
     * @param {Node} other
     * The node to compare the reference node to.
     * @returns {number}
     * Returns how the node is positioned relatively to the reference node according to the
     * bitmask. 0 if reference node and given node are the same.
     * @since DOM Level 3
     * @see https://www.w3.org/TR/2004/REC-DOM-Level-3-Core-20040407/core.html#Node3-compare
     * @see https://dom.spec.whatwg.org/#dom-node-comparedocumentposition
     */
    compareDocumentPosition: function(e) {
      if (this === e) return 0;
      var r = e, i = this, a = null, d = null;
      if (r instanceof De && (a = r, r = a.ownerElement), i instanceof De && (d = i, i = d.ownerElement, a && r && i === r))
        for (var N = 0, z; z = i.attributes[N]; N++) {
          if (z === a)
            return S.DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC + S.DOCUMENT_POSITION_PRECEDING;
          if (z === d)
            return S.DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC + S.DOCUMENT_POSITION_FOLLOWING;
        }
      if (!r || !i || i.ownerDocument !== r.ownerDocument)
        return S.DOCUMENT_POSITION_DISCONNECTED + S.DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC + (O(i.ownerDocument) > O(r.ownerDocument) ? S.DOCUMENT_POSITION_FOLLOWING : S.DOCUMENT_POSITION_PRECEDING);
      if (d && r === i)
        return S.DOCUMENT_POSITION_CONTAINS + S.DOCUMENT_POSITION_PRECEDING;
      if (a && r === i)
        return S.DOCUMENT_POSITION_CONTAINED_BY + S.DOCUMENT_POSITION_FOLLOWING;
      for (var se = [], he = r.parentNode; he; ) {
        if (!d && he === i)
          return S.DOCUMENT_POSITION_CONTAINED_BY + S.DOCUMENT_POSITION_FOLLOWING;
        se.push(he), he = he.parentNode;
      }
      se.reverse();
      for (var ve = [], me = i.parentNode; me; ) {
        if (!a && me === r)
          return S.DOCUMENT_POSITION_CONTAINS + S.DOCUMENT_POSITION_PRECEDING;
        ve.push(me), me = me.parentNode;
      }
      ve.reverse();
      var Xe = V(se, ve);
      for (var Ie in Xe.childNodes) {
        var be = Xe.childNodes[Ie];
        if (be === i) return S.DOCUMENT_POSITION_FOLLOWING;
        if (be === r) return S.DOCUMENT_POSITION_PRECEDING;
        if (ve.indexOf(be) >= 0) return S.DOCUMENT_POSITION_FOLLOWING;
        if (se.indexOf(be) >= 0) return S.DOCUMENT_POSITION_PRECEDING;
      }
      return 0;
    }
  };
  function Fe(e) {
    return e == "<" && "&lt;" || e == ">" && "&gt;" || e == "&" && "&amp;" || e == '"' && "&quot;" || "&#" + e.charCodeAt() + ";";
  }
  m(I, R), m(I, R.prototype), m(S, R), m(S, R.prototype);
  function ye(e, r) {
    if (r(e))
      return !0;
    if (e = e.firstChild)
      do
        if (ye(e, r))
          return !0;
      while (e = e.nextSibling);
  }
  function ce(e, r) {
    Y(e);
    var i = r || {};
    this.ownerDocument = this, this.contentType = i.contentType || C.XML_APPLICATION, this.type = c(this.contentType) ? "html" : "xml";
  }
  function Le(e, r, i) {
    e && e._inc++;
    var a = i.namespaceURI;
    a === f.XMLNS && (r._nsMap[i.prefix ? i.localName : ""] = i.value);
  }
  function _e(e, r, i, a) {
    e && e._inc++;
    var d = i.namespaceURI;
    d === f.XMLNS && delete r._nsMap[i.prefix ? i.localName : ""];
  }
  function Ce(e, r, i) {
    if (e && e._inc) {
      e._inc++;
      var a = r.childNodes;
      if (i && !i.nextSibling)
        a[a.length++] = i;
      else {
        for (var d = r.firstChild, N = 0; d; )
          a[N++] = d, d = d.nextSibling;
        a.length = N, delete a[a.length];
      }
    }
  }
  function Re(e, r) {
    if (e !== r.parentNode)
      throw new p(p.NOT_FOUND_ERR, "child's parent is not parent");
    var i = r.previousSibling, a = r.nextSibling;
    return i ? i.nextSibling = a : e.firstChild = a, a ? a.previousSibling = i : e.lastChild = i, Ce(e.ownerDocument, e), r.parentNode = null, r.previousSibling = null, r.nextSibling = null, r;
  }
  function Ue(e) {
    return e && (e.nodeType === R.DOCUMENT_NODE || e.nodeType === R.DOCUMENT_FRAGMENT_NODE || e.nodeType === R.ELEMENT_NODE);
  }
  function qe(e) {
    return e && (e.nodeType === R.CDATA_SECTION_NODE || e.nodeType === R.COMMENT_NODE || e.nodeType === R.DOCUMENT_FRAGMENT_NODE || e.nodeType === R.DOCUMENT_TYPE_NODE || e.nodeType === R.ELEMENT_NODE || e.nodeType === R.PROCESSING_INSTRUCTION_NODE || e.nodeType === R.TEXT_NODE);
  }
  function Ae(e) {
    return e && e.nodeType === R.DOCUMENT_TYPE_NODE;
  }
  function j(e) {
    return e && e.nodeType === R.ELEMENT_NODE;
  }
  function Pe(e) {
    return e && e.nodeType === R.TEXT_NODE;
  }
  function ne(e, r) {
    var i = e.childNodes || [];
    if (t(i, j) || Ae(r))
      return !1;
    var a = t(i, Ae);
    return !(r && a && i.indexOf(a) > i.indexOf(r));
  }
  function Ge(e, r) {
    var i = e.childNodes || [];
    function a(N) {
      return j(N) && N !== r;
    }
    if (t(i, a))
      return !1;
    var d = t(i, Ae);
    return !(r && d && i.indexOf(d) > i.indexOf(r));
  }
  function at(e, r, i) {
    if (!Ue(e))
      throw new p(p.HIERARCHY_REQUEST_ERR, "Unexpected parent node type " + e.nodeType);
    if (i && i.parentNode !== e)
      throw new p(p.NOT_FOUND_ERR, "child not in parent");
    if (
      // 4. If `node` is not a DocumentFragment, DocumentType, Element, or CharacterData node, then throw a "HierarchyRequestError" DOMException.
      !qe(r) || // 5. If either `node` is a Text node and `parent` is a document,
      // the sax parser currently adds top level text nodes, this will be fixed in 0.9.0
      // || (node.nodeType === Node.TEXT_NODE && parent.nodeType === Node.DOCUMENT_NODE)
      // or `node` is a doctype and `parent` is not a document, then throw a "HierarchyRequestError" DOMException.
      Ae(r) && e.nodeType !== R.DOCUMENT_NODE
    )
      throw new p(
        p.HIERARCHY_REQUEST_ERR,
        "Unexpected node type " + r.nodeType + " for parent node type " + e.nodeType
      );
  }
  function Ve(e, r, i) {
    var a = e.childNodes || [], d = r.childNodes || [];
    if (r.nodeType === R.DOCUMENT_FRAGMENT_NODE) {
      var N = d.filter(j);
      if (N.length > 1 || t(d, Pe))
        throw new p(p.HIERARCHY_REQUEST_ERR, "More than one element or text in fragment");
      if (N.length === 1 && !ne(e, i))
        throw new p(p.HIERARCHY_REQUEST_ERR, "Element in fragment can not be inserted before doctype");
    }
    if (j(r) && !ne(e, i))
      throw new p(p.HIERARCHY_REQUEST_ERR, "Only one element can be added and only after doctype");
    if (Ae(r)) {
      if (t(a, Ae))
        throw new p(p.HIERARCHY_REQUEST_ERR, "Only one doctype is allowed");
      var z = t(a, j);
      if (i && a.indexOf(z) < a.indexOf(i))
        throw new p(p.HIERARCHY_REQUEST_ERR, "Doctype can only be inserted before an element");
      if (!i && z)
        throw new p(p.HIERARCHY_REQUEST_ERR, "Doctype can not be appended since element is present");
    }
  }
  function ze(e, r, i) {
    var a = e.childNodes || [], d = r.childNodes || [];
    if (r.nodeType === R.DOCUMENT_FRAGMENT_NODE) {
      var N = d.filter(j);
      if (N.length > 1 || t(d, Pe))
        throw new p(p.HIERARCHY_REQUEST_ERR, "More than one element or text in fragment");
      if (N.length === 1 && !Ge(e, i))
        throw new p(p.HIERARCHY_REQUEST_ERR, "Element in fragment can not be inserted before doctype");
    }
    if (j(r) && !Ge(e, i))
      throw new p(p.HIERARCHY_REQUEST_ERR, "Only one element can be added and only after doctype");
    if (Ae(r)) {
      if (t(a, function(he) {
        return Ae(he) && he !== i;
      }))
        throw new p(p.HIERARCHY_REQUEST_ERR, "Only one doctype is allowed");
      var z = t(a, j);
      if (i && a.indexOf(z) < a.indexOf(i))
        throw new p(p.HIERARCHY_REQUEST_ERR, "Doctype can only be inserted before an element");
    }
  }
  function F(e, r, i, a) {
    at(e, r, i), e.nodeType === R.DOCUMENT_NODE && (a || Ve)(e, r, i);
    var d = r.parentNode;
    if (d && d.removeChild(r), r.nodeType === y) {
      var N = r.firstChild;
      if (N == null)
        return r;
      var z = r.lastChild;
    } else
      N = z = r;
    var se = i ? i.previousSibling : e.lastChild;
    N.previousSibling = se, z.nextSibling = i, se ? se.nextSibling = N : e.firstChild = N, i == null ? e.lastChild = z : i.previousSibling = z;
    do
      N.parentNode = e;
    while (N !== z && (N = N.nextSibling));
    return Ce(e.ownerDocument || e, e, r), r.nodeType == y && (r.firstChild = r.lastChild = null), r;
  }
  ce.prototype = {
    /**
     * The implementation that created this document.
     *
     * @type DOMImplementation
     * @readonly
     */
    implementation: null,
    nodeName: "#document",
    nodeType: b,
    /**
     * The DocumentType node of the document.
     *
     * @type DocumentType
     * @readonly
     */
    doctype: null,
    documentElement: null,
    _inc: 1,
    insertBefore: function(e, r) {
      if (e.nodeType === y) {
        for (var i = e.firstChild; i; ) {
          var a = i.nextSibling;
          this.insertBefore(i, r), i = a;
        }
        return e;
      }
      return F(this, e, r), e.ownerDocument = this, this.documentElement === null && e.nodeType === U && (this.documentElement = e), e;
    },
    removeChild: function(e) {
      var r = Re(this, e);
      return r === this.documentElement && (this.documentElement = null), r;
    },
    replaceChild: function(e, r) {
      F(this, e, r, ze), e.ownerDocument = this, r && this.removeChild(r), j(e) && (this.documentElement = e);
    },
    // Introduced in DOM Level 2:
    importNode: function(e, r) {
      return Tt(this, e, r);
    },
    // Introduced in DOM Level 2:
    getElementById: function(e) {
      var r = null;
      return ye(this.documentElement, function(i) {
        if (i.nodeType == U && i.getAttribute("id") == e)
          return r = i, !0;
      }), r;
    },
    /**
     * Creates a new `Element` that is owned by this `Document`.
     * In HTML Documents `localName` is the lower cased `tagName`,
     * otherwise no transformation is being applied.
     * When `contentType` implies the HTML namespace, it will be set as `namespaceURI`.
     *
     * __This implementation differs from the specification:__ - The provided name is not checked
     * against the `Name` production,
     * so no related error will be thrown.
     * - There is no interface `HTMLElement`, it is always an `Element`.
     * - There is no support for a second argument to indicate using custom elements.
     *
     * @param {string} tagName
     * @returns {Element}
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Document/createElement
     * @see https://dom.spec.whatwg.org/#dom-document-createelement
     * @see https://dom.spec.whatwg.org/#concept-create-element
     */
    createElement: function(e) {
      var r = new Q(g);
      r.ownerDocument = this, this.type === "html" && (e = e.toLowerCase()), u(this.contentType) && (r.namespaceURI = f.HTML), r.nodeName = e, r.tagName = e, r.localName = e, r.childNodes = new T();
      var i = r.attributes = new P();
      return i._ownerElement = r, r;
    },
    /**
     * @returns {DocumentFragment}
     */
    createDocumentFragment: function() {
      var e = new Ye(g);
      return e.ownerDocument = this, e.childNodes = new T(), e;
    },
    /**
     * @param {string} data
     * @returns {Text}
     */
    createTextNode: function(e) {
      var r = new He(g);
      return r.ownerDocument = this, r.childNodes = new T(), r.appendData(e), r;
    },
    /**
     * @param {string} data
     * @returns {Comment}
     */
    createComment: function(e) {
      var r = new Je(g);
      return r.ownerDocument = this, r.childNodes = new T(), r.appendData(e), r;
    },
    /**
     * @param {string} data
     * @returns {CDATASection}
     */
    createCDATASection: function(e) {
      var r = new Ze(g);
      return r.ownerDocument = this, r.childNodes = new T(), r.appendData(e), r;
    },
    /**
     * @param {string} target
     * @param {string} data
     * @returns {ProcessingInstruction}
     */
    createProcessingInstruction: function(e, r) {
      var i = new tt(g);
      return i.ownerDocument = this, i.childNodes = new T(), i.nodeName = i.target = e, i.nodeValue = i.data = r, i;
    },
    /**
     * Creates an `Attr` node that is owned by this document.
     * In HTML Documents `localName` is the lower cased `name`,
     * otherwise no transformation is being applied.
     *
     * __This implementation differs from the specification:__ - The provided name is not checked
     * against the `Name` production,
     * so no related error will be thrown.
     *
     * @param {string} name
     * @returns {Attr}
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Document/createAttribute
     * @see https://dom.spec.whatwg.org/#dom-document-createattribute
     */
    createAttribute: function(e) {
      if (!k.QName_exact.test(e))
        throw new p(p.INVALID_CHARACTER_ERR, 'invalid character in name "' + e + '"');
      return this.type === "html" && (e = e.toLowerCase()), this._createAttribute(e);
    },
    _createAttribute: function(e) {
      var r = new De(g);
      return r.ownerDocument = this, r.childNodes = new T(), r.name = e, r.nodeName = e, r.localName = e, r.specified = !0, r;
    },
    /**
     * Creates an EntityReference object.
     * The current implementation does not fill the `childNodes` with those of the corresponding
     * `Entity`
     *
     * @deprecated
     * In DOM Level 4.
     * @param {string} name
     * The name of the entity to reference. No namespace well-formedness checks are performed.
     * @returns {EntityReference}
     * @throws {DOMException}
     * With code `INVALID_CHARACTER_ERR` when `name` is not valid.
     * @throws {DOMException}
     * with code `NOT_SUPPORTED_ERR` when the document is of type `html`
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#ID-392B75AE
     */
    createEntityReference: function(e) {
      if (!k.Name.test(e))
        throw new p(p.INVALID_CHARACTER_ERR, 'not a valid xml name "' + e + '"');
      if (this.type === "html")
        throw new p("document is an html document", B.NotSupportedError);
      var r = new et(g);
      return r.ownerDocument = this, r.childNodes = new T(), r.nodeName = e, r;
    },
    // Introduced in DOM Level 2:
    /**
     * @param {string} namespaceURI
     * @param {string} qualifiedName
     * @returns {Element}
     */
    createElementNS: function(e, r) {
      var i = de(e, r), a = new Q(g), d = a.attributes = new P();
      return a.childNodes = new T(), a.ownerDocument = this, a.nodeName = r, a.tagName = r, a.namespaceURI = i[0], a.prefix = i[1], a.localName = i[2], d._ownerElement = a, a;
    },
    // Introduced in DOM Level 2:
    /**
     * @param {string} namespaceURI
     * @param {string} qualifiedName
     * @returns {Attr}
     */
    createAttributeNS: function(e, r) {
      var i = de(e, r), a = new De(g);
      return a.ownerDocument = this, a.childNodes = new T(), a.nodeName = r, a.name = r, a.specified = !0, a.namespaceURI = i[0], a.prefix = i[1], a.localName = i[2], a;
    }
  }, _(ce, R);
  function Q(e) {
    Y(e), this._nsMap = /* @__PURE__ */ Object.create(null);
  }
  Q.prototype = {
    nodeType: U,
    /**
     * The attributes of this element.
     *
     * @type {NamedNodeMap | null}
     */
    attributes: null,
    getQualifiedName: function() {
      return this.prefix ? this.prefix + ":" + this.localName : this.localName;
    },
    _isInHTMLDocumentAndNamespace: function() {
      return this.ownerDocument.type === "html" && this.namespaceURI === f.HTML;
    },
    /**
     * Implementaton of Level2 Core function hasAttributes.
     *
     * @returns {boolean}
     * True if attribute list is not empty.
     * @see https://www.w3.org/TR/DOM-Level-2-Core/#core-ID-NodeHasAttrs
     */
    hasAttributes: function() {
      return !!(this.attributes && this.attributes.length);
    },
    hasAttribute: function(e) {
      return !!this.getAttributeNode(e);
    },
    /**
     * Returns element’s first attribute whose qualified name is `name`, and `null`
     * if there is no such attribute.
     *
     * @param {string} name
     * @returns {string | null}
     */
    getAttribute: function(e) {
      var r = this.getAttributeNode(e);
      return r ? r.value : null;
    },
    getAttributeNode: function(e) {
      return this._isInHTMLDocumentAndNamespace() && (e = e.toLowerCase()), this.attributes.getNamedItem(e);
    },
    /**
     * Sets the value of element’s first attribute whose qualified name is qualifiedName to value.
     *
     * @param {string} name
     * @param {string} value
     */
    setAttribute: function(e, r) {
      this._isInHTMLDocumentAndNamespace() && (e = e.toLowerCase());
      var i = this.getAttributeNode(e);
      i ? i.value = i.nodeValue = "" + r : (i = this.ownerDocument._createAttribute(e), i.value = i.nodeValue = "" + r, this.setAttributeNode(i));
    },
    removeAttribute: function(e) {
      var r = this.getAttributeNode(e);
      r && this.removeAttributeNode(r);
    },
    setAttributeNode: function(e) {
      return this.attributes.setNamedItem(e);
    },
    setAttributeNodeNS: function(e) {
      return this.attributes.setNamedItemNS(e);
    },
    removeAttributeNode: function(e) {
      return this.attributes.removeNamedItem(e.nodeName);
    },
    //get real attribute name,and remove it by removeAttributeNode
    removeAttributeNS: function(e, r) {
      var i = this.getAttributeNodeNS(e, r);
      i && this.removeAttributeNode(i);
    },
    hasAttributeNS: function(e, r) {
      return this.getAttributeNodeNS(e, r) != null;
    },
    /**
     * Returns element’s attribute whose namespace is `namespaceURI` and local name is
     * `localName`,
     * or `null` if there is no such attribute.
     *
     * @param {string} namespaceURI
     * @param {string} localName
     * @returns {string | null}
     */
    getAttributeNS: function(e, r) {
      var i = this.getAttributeNodeNS(e, r);
      return i ? i.value : null;
    },
    /**
     * Sets the value of element’s attribute whose namespace is `namespaceURI` and local name is
     * `localName` to value.
     *
     * @param {string} namespaceURI
     * @param {string} qualifiedName
     * @param {string} value
     * @see https://dom.spec.whatwg.org/#dom-element-setattributens
     */
    setAttributeNS: function(e, r, i) {
      var a = de(e, r), d = a[2], N = this.getAttributeNodeNS(e, d);
      N ? N.value = N.nodeValue = "" + i : (N = this.ownerDocument.createAttributeNS(e, r), N.value = N.nodeValue = "" + i, this.setAttributeNode(N));
    },
    getAttributeNodeNS: function(e, r) {
      return this.attributes.getNamedItemNS(e, r);
    },
    /**
     * Returns a LiveNodeList of all child elements which have **all** of the given class name(s).
     *
     * Returns an empty list if `classNames` is an empty string or only contains HTML white space
     * characters.
     *
     * Warning: This returns a live LiveNodeList.
     * Changes in the DOM will reflect in the array as the changes occur.
     * If an element selected by this array no longer qualifies for the selector,
     * it will automatically be removed. Be aware of this for iteration purposes.
     *
     * @param {string} classNames
     * Is a string representing the class name(s) to match; multiple class names are separated by
     * (ASCII-)whitespace.
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Element/getElementsByClassName
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Document/getElementsByClassName
     * @see https://dom.spec.whatwg.org/#concept-getelementsbyclassname
     */
    getElementsByClassName: function(e) {
      var r = q(e);
      return new w(this, function(i) {
        var a = [];
        return r.length > 0 && ye(i, function(d) {
          if (d !== i && d.nodeType === U) {
            var N = d.getAttribute("class");
            if (N) {
              var z = e === N;
              if (!z) {
                var se = q(N);
                z = r.every(W(se));
              }
              z && a.push(d);
            }
          }
        }), a;
      });
    },
    /**
     * Returns a LiveNodeList of elements with the given qualifiedName.
     * Searching for all descendants can be done by passing `*` as `qualifiedName`.
     *
     * All descendants of the specified element are searched, but not the element itself.
     * The returned list is live, which means it updates itself with the DOM tree automatically.
     * Therefore, there is no need to call `Element.getElementsByTagName()`
     * with the same element and arguments repeatedly if the DOM changes in between calls.
     *
     * When called on an HTML element in an HTML document,
     * `getElementsByTagName` lower-cases the argument before searching for it.
     * This is undesirable when trying to match camel-cased SVG elements (such as
     * `<linearGradient>`) in an HTML document.
     * Instead, use `Element.getElementsByTagNameNS()`,
     * which preserves the capitalization of the tag name.
     *
     * `Element.getElementsByTagName` is similar to `Document.getElementsByTagName()`,
     * except that it only searches for elements that are descendants of the specified element.
     *
     * @param {string} qualifiedName
     * @returns {LiveNodeList}
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Element/getElementsByTagName
     * @see https://dom.spec.whatwg.org/#concept-getelementsbytagname
     */
    getElementsByTagName: function(e) {
      var r = (this.nodeType === b ? this : this.ownerDocument).type === "html", i = e.toLowerCase();
      return new w(this, function(a) {
        var d = [];
        return ye(a, function(N) {
          if (!(N === a || N.nodeType !== U))
            if (e === "*")
              d.push(N);
            else {
              var z = N.getQualifiedName(), se = r && N.namespaceURI === f.HTML ? i : e;
              z === se && d.push(N);
            }
        }), d;
      });
    },
    getElementsByTagNameNS: function(e, r) {
      return new w(this, function(i) {
        var a = [];
        return ye(i, function(d) {
          d !== i && d.nodeType === U && (e === "*" || d.namespaceURI === e) && (r === "*" || d.localName == r) && a.push(d);
        }), a;
      });
    }
  }, ce.prototype.getElementsByClassName = Q.prototype.getElementsByClassName, ce.prototype.getElementsByTagName = Q.prototype.getElementsByTagName, ce.prototype.getElementsByTagNameNS = Q.prototype.getElementsByTagNameNS, _(Q, R);
  function De(e) {
    Y(e), this.namespaceURI = null, this.prefix = null, this.ownerElement = null;
  }
  De.prototype.nodeType = $, _(De, R);
  function ge(e) {
    Y(e);
  }
  ge.prototype = {
    data: "",
    substringData: function(e, r) {
      return this.data.substring(e, e + r);
    },
    appendData: function(e) {
      e = this.data + e, this.nodeValue = this.data = e, this.length = e.length;
    },
    insertData: function(e, r) {
      this.replaceData(e, 0, r);
    },
    deleteData: function(e, r) {
      this.replaceData(e, r, "");
    },
    replaceData: function(e, r, i) {
      var a = this.data.substring(0, e), d = this.data.substring(e + r);
      i = a + i + d, this.nodeValue = this.data = i, this.length = i.length;
    }
  }, _(ge, R);
  function He(e) {
    Y(e);
  }
  He.prototype = {
    nodeName: "#text",
    nodeType: J,
    splitText: function(e) {
      var r = this.data, i = r.substring(e);
      r = r.substring(0, e), this.data = this.nodeValue = r, this.length = r.length;
      var a = this.ownerDocument.createTextNode(i);
      return this.parentNode && this.parentNode.insertBefore(a, this.nextSibling), a;
    }
  }, _(He, ge);
  function Je(e) {
    Y(e);
  }
  Je.prototype = {
    nodeName: "#comment",
    nodeType: A
  }, _(Je, ge);
  function Ze(e) {
    Y(e);
  }
  Ze.prototype = {
    nodeName: "#cdata-section",
    nodeType: Ne
  }, _(Ze, He);
  function Ke(e) {
    Y(e);
  }
  Ke.prototype.nodeType = v, _(Ke, R);
  function ot(e) {
    Y(e);
  }
  ot.prototype.nodeType = E, _(ot, R);
  function ct(e) {
    Y(e);
  }
  ct.prototype.nodeType = we, _(ct, R);
  function et(e) {
    Y(e);
  }
  et.prototype.nodeType = Z, _(et, R);
  function Ye(e) {
    Y(e);
  }
  Ye.prototype.nodeName = "#document-fragment", Ye.prototype.nodeType = y, _(Ye, R);
  function tt(e) {
    Y(e);
  }
  tt.prototype.nodeType = l, _(tt, ge);
  function dt() {
  }
  dt.prototype.serializeToString = function(e, r) {
    return ee.call(e, r);
  }, R.prototype.toString = ee;
  function ee(e) {
    var r = [], i = this.nodeType === b && this.documentElement || this, a = i.prefix, d = i.namespaceURI;
    if (d && a == null) {
      var a = i.lookupPrefix(d);
      if (a == null)
        var N = [
          { namespace: d, prefix: null }
          //{namespace:uri,prefix:''}
        ];
    }
    return xe(this, r, e, N), r.join("");
  }
  function le(e, r, i) {
    var a = e.prefix || "", d = e.namespaceURI;
    if (!d || a === "xml" && d === f.XML || d === f.XMLNS)
      return !1;
    for (var N = i.length; N--; ) {
      var z = i[N];
      if (z.prefix === a)
        return z.namespace !== d;
    }
    return !0;
  }
  function Me(e, r, i) {
    e.push(" ", r, '="', i.replace(/[<>&"\t\n\r]/g, Fe), '"');
  }
  function xe(e, r, i, a) {
    a || (a = []);
    var d = e.nodeType === b ? e : e.ownerDocument, N = d.type === "html";
    if (i)
      if (e = i(e), e) {
        if (typeof e == "string") {
          r.push(e);
          return;
        }
      } else
        return;
    switch (e.nodeType) {
      case U:
        var z = e.attributes, se = z.length, pe = e.firstChild, he = e.tagName, ve = he;
        if (!N && !e.prefix && e.namespaceURI) {
          for (var me, Xe = 0; Xe < z.length; Xe++)
            if (z.item(Xe).name === "xmlns") {
              me = z.item(Xe).value;
              break;
            }
          if (!me)
            for (var Ie = a.length - 1; Ie >= 0; Ie--) {
              var be = a[Ie];
              if (be.prefix === "" && be.namespace === e.namespaceURI) {
                me = be.namespace;
                break;
              }
            }
          if (me !== e.namespaceURI)
            for (var Ie = a.length - 1; Ie >= 0; Ie--) {
              var be = a[Ie];
              if (be.namespace === e.namespaceURI) {
                be.prefix && (ve = be.prefix + ":" + he);
                break;
              }
            }
        }
        r.push("<", ve);
        for (var $e = 0; $e < se; $e++) {
          var Se = z.item($e);
          Se.prefix == "xmlns" ? a.push({
            prefix: Se.localName,
            namespace: Se.value
          }) : Se.nodeName == "xmlns" && a.push({ prefix: "", namespace: Se.value });
        }
        for (var $e = 0; $e < se; $e++) {
          var Se = z.item($e);
          if (le(Se, N, a)) {
            var je = Se.prefix || "", lt = Se.namespaceURI;
            Me(r, je ? "xmlns:" + je : "xmlns", lt), a.push({ prefix: je, namespace: lt });
          }
          xe(Se, r, i, a);
        }
        if (he === ve && le(e, N, a)) {
          var je = e.prefix || "", lt = e.namespaceURI;
          Me(r, je ? "xmlns:" + je : "xmlns", lt), a.push({ prefix: je, namespace: lt });
        }
        var Dt = !pe;
        if (Dt && (N || e.namespaceURI === f.HTML) && (Dt = h(he)), Dt)
          r.push("/>");
        else {
          if (r.push(">"), N && o(he))
            for (; pe; )
              pe.data ? r.push(pe.data) : xe(pe, r, i, a.slice()), pe = pe.nextSibling;
          else
            for (; pe; )
              xe(pe, r, i, a.slice()), pe = pe.nextSibling;
          r.push("</", ve, ">");
        }
        return;
      case b:
      case y:
        for (var pe = e.firstChild; pe; )
          xe(pe, r, i, a.slice()), pe = pe.nextSibling;
        return;
      case $:
        return Me(r, e.name, e.value);
      case J:
        return r.push(e.data.replace(/[<&>]/g, Fe));
      case Ne:
        return r.push(k.CDATA_START, e.data, k.CDATA_END);
      case A:
        return r.push(k.COMMENT_START, e.data, k.COMMENT_END);
      case v:
        var bt = e.publicId, rt = e.systemId;
        r.push(k.DOCTYPE_DECL_START, " ", e.name), bt ? (r.push(" ", k.PUBLIC, " ", bt), rt && rt !== "." && r.push(" ", rt)) : rt && rt !== "." && r.push(" ", k.SYSTEM, " ", rt), e.internalSubset && r.push(" [", e.internalSubset, "]"), r.push(">");
        return;
      case l:
        return r.push("<?", e.target, " ", e.data, "?>");
      case Z:
        return r.push("&", e.nodeName, ";");
      //case ENTITY_NODE:
      //case NOTATION_NODE:
      default:
        r.push("??", e.nodeName);
    }
  }
  function Tt(e, r, i) {
    var a;
    switch (r.nodeType) {
      case U:
        a = r.cloneNode(!1), a.ownerDocument = e;
      //var attrs = node2.attributes;
      //var len = attrs.length;
      //for(var i=0;i<len;i++){
      //node2.setAttributeNodeNS(importNode(doc,attrs.item(i),deep));
      //}
      case y:
        break;
      case $:
        i = !0;
        break;
    }
    if (a || (a = r.cloneNode(!1)), a.ownerDocument = e, a.parentNode = null, i)
      for (var d = r.firstChild; d; )
        a.appendChild(Tt(e, d, i)), d = d.nextSibling;
    return a;
  }
  function Et(e, r, i) {
    var a = new r.constructor(g);
    for (var d in r)
      if (s(r, d)) {
        var N = r[d];
        typeof N != "object" && N != a[d] && (a[d] = N);
      }
    switch (r.childNodes && (a.childNodes = new T()), a.ownerDocument = e, a.nodeType) {
      case U:
        var z = r.attributes, se = a.attributes = new P(), he = z.length;
        se._ownerElement = a;
        for (var ve = 0; ve < he; ve++)
          a.setAttributeNode(Et(e, z.item(ve), !0));
        break;
      case $:
        i = !0;
    }
    if (i)
      for (var me = r.firstChild; me; )
        a.appendChild(Et(e, me, i)), me = me.nextSibling;
    return a;
  }
  function Ct(e, r, i) {
    e[r] = i;
  }
  try {
    if (Object.defineProperty) {
      let e = function(r) {
        switch (r.nodeType) {
          case U:
          case y:
            var i = [];
            for (r = r.firstChild; r; )
              r.nodeType !== 7 && r.nodeType !== 8 && i.push(e(r)), r = r.nextSibling;
            return i.join("");
          default:
            return r.nodeValue;
        }
      };
      Object.defineProperty(w.prototype, "length", {
        get: function() {
          return x(this), this.$$length;
        }
      }), Object.defineProperty(R.prototype, "textContent", {
        get: function() {
          return e(this);
        },
        set: function(r) {
          switch (this.nodeType) {
            case U:
            case y:
              for (; this.firstChild; )
                this.removeChild(this.firstChild);
              (r || String(r)) && this.appendChild(this.ownerDocument.createTextNode(r));
              break;
            default:
              this.data = r, this.value = r, this.nodeValue = r;
          }
        }
      }), Ct = function(r, i, a) {
        r["$$" + i] = a;
      };
    }
  } catch {
  }
  return K._updateLiveList = x, K.Attr = De, K.CDATASection = Ze, K.CharacterData = ge, K.Comment = Je, K.Document = ce, K.DocumentFragment = Ye, K.DocumentType = Ke, K.DOMImplementation = ie, K.Element = Q, K.Entity = ct, K.EntityReference = et, K.LiveNodeList = w, K.NamedNodeMap = P, K.Node = R, K.NodeList = T, K.Notation = ot, K.Text = He, K.ProcessingInstruction = tt, K.XMLSerializer = dt, K;
}
var We = {}, gt = {}, Ot;
function Dr() {
  return Ot || (Ot = 1, function(n) {
    var t = st().freeze;
    n.XML_ENTITIES = t({
      amp: "&",
      apos: "'",
      gt: ">",
      lt: "<",
      quot: '"'
    }), n.HTML_ENTITIES = t({
      Aacute: "Á",
      aacute: "á",
      Abreve: "Ă",
      abreve: "ă",
      ac: "∾",
      acd: "∿",
      acE: "∾̳",
      Acirc: "Â",
      acirc: "â",
      acute: "´",
      Acy: "А",
      acy: "а",
      AElig: "Æ",
      aelig: "æ",
      af: "⁡",
      Afr: "𝔄",
      afr: "𝔞",
      Agrave: "À",
      agrave: "à",
      alefsym: "ℵ",
      aleph: "ℵ",
      Alpha: "Α",
      alpha: "α",
      Amacr: "Ā",
      amacr: "ā",
      amalg: "⨿",
      AMP: "&",
      amp: "&",
      And: "⩓",
      and: "∧",
      andand: "⩕",
      andd: "⩜",
      andslope: "⩘",
      andv: "⩚",
      ang: "∠",
      ange: "⦤",
      angle: "∠",
      angmsd: "∡",
      angmsdaa: "⦨",
      angmsdab: "⦩",
      angmsdac: "⦪",
      angmsdad: "⦫",
      angmsdae: "⦬",
      angmsdaf: "⦭",
      angmsdag: "⦮",
      angmsdah: "⦯",
      angrt: "∟",
      angrtvb: "⊾",
      angrtvbd: "⦝",
      angsph: "∢",
      angst: "Å",
      angzarr: "⍼",
      Aogon: "Ą",
      aogon: "ą",
      Aopf: "𝔸",
      aopf: "𝕒",
      ap: "≈",
      apacir: "⩯",
      apE: "⩰",
      ape: "≊",
      apid: "≋",
      apos: "'",
      ApplyFunction: "⁡",
      approx: "≈",
      approxeq: "≊",
      Aring: "Å",
      aring: "å",
      Ascr: "𝒜",
      ascr: "𝒶",
      Assign: "≔",
      ast: "*",
      asymp: "≈",
      asympeq: "≍",
      Atilde: "Ã",
      atilde: "ã",
      Auml: "Ä",
      auml: "ä",
      awconint: "∳",
      awint: "⨑",
      backcong: "≌",
      backepsilon: "϶",
      backprime: "‵",
      backsim: "∽",
      backsimeq: "⋍",
      Backslash: "∖",
      Barv: "⫧",
      barvee: "⊽",
      Barwed: "⌆",
      barwed: "⌅",
      barwedge: "⌅",
      bbrk: "⎵",
      bbrktbrk: "⎶",
      bcong: "≌",
      Bcy: "Б",
      bcy: "б",
      bdquo: "„",
      becaus: "∵",
      Because: "∵",
      because: "∵",
      bemptyv: "⦰",
      bepsi: "϶",
      bernou: "ℬ",
      Bernoullis: "ℬ",
      Beta: "Β",
      beta: "β",
      beth: "ℶ",
      between: "≬",
      Bfr: "𝔅",
      bfr: "𝔟",
      bigcap: "⋂",
      bigcirc: "◯",
      bigcup: "⋃",
      bigodot: "⨀",
      bigoplus: "⨁",
      bigotimes: "⨂",
      bigsqcup: "⨆",
      bigstar: "★",
      bigtriangledown: "▽",
      bigtriangleup: "△",
      biguplus: "⨄",
      bigvee: "⋁",
      bigwedge: "⋀",
      bkarow: "⤍",
      blacklozenge: "⧫",
      blacksquare: "▪",
      blacktriangle: "▴",
      blacktriangledown: "▾",
      blacktriangleleft: "◂",
      blacktriangleright: "▸",
      blank: "␣",
      blk12: "▒",
      blk14: "░",
      blk34: "▓",
      block: "█",
      bne: "=⃥",
      bnequiv: "≡⃥",
      bNot: "⫭",
      bnot: "⌐",
      Bopf: "𝔹",
      bopf: "𝕓",
      bot: "⊥",
      bottom: "⊥",
      bowtie: "⋈",
      boxbox: "⧉",
      boxDL: "╗",
      boxDl: "╖",
      boxdL: "╕",
      boxdl: "┐",
      boxDR: "╔",
      boxDr: "╓",
      boxdR: "╒",
      boxdr: "┌",
      boxH: "═",
      boxh: "─",
      boxHD: "╦",
      boxHd: "╤",
      boxhD: "╥",
      boxhd: "┬",
      boxHU: "╩",
      boxHu: "╧",
      boxhU: "╨",
      boxhu: "┴",
      boxminus: "⊟",
      boxplus: "⊞",
      boxtimes: "⊠",
      boxUL: "╝",
      boxUl: "╜",
      boxuL: "╛",
      boxul: "┘",
      boxUR: "╚",
      boxUr: "╙",
      boxuR: "╘",
      boxur: "└",
      boxV: "║",
      boxv: "│",
      boxVH: "╬",
      boxVh: "╫",
      boxvH: "╪",
      boxvh: "┼",
      boxVL: "╣",
      boxVl: "╢",
      boxvL: "╡",
      boxvl: "┤",
      boxVR: "╠",
      boxVr: "╟",
      boxvR: "╞",
      boxvr: "├",
      bprime: "‵",
      Breve: "˘",
      breve: "˘",
      brvbar: "¦",
      Bscr: "ℬ",
      bscr: "𝒷",
      bsemi: "⁏",
      bsim: "∽",
      bsime: "⋍",
      bsol: "\\",
      bsolb: "⧅",
      bsolhsub: "⟈",
      bull: "•",
      bullet: "•",
      bump: "≎",
      bumpE: "⪮",
      bumpe: "≏",
      Bumpeq: "≎",
      bumpeq: "≏",
      Cacute: "Ć",
      cacute: "ć",
      Cap: "⋒",
      cap: "∩",
      capand: "⩄",
      capbrcup: "⩉",
      capcap: "⩋",
      capcup: "⩇",
      capdot: "⩀",
      CapitalDifferentialD: "ⅅ",
      caps: "∩︀",
      caret: "⁁",
      caron: "ˇ",
      Cayleys: "ℭ",
      ccaps: "⩍",
      Ccaron: "Č",
      ccaron: "č",
      Ccedil: "Ç",
      ccedil: "ç",
      Ccirc: "Ĉ",
      ccirc: "ĉ",
      Cconint: "∰",
      ccups: "⩌",
      ccupssm: "⩐",
      Cdot: "Ċ",
      cdot: "ċ",
      cedil: "¸",
      Cedilla: "¸",
      cemptyv: "⦲",
      cent: "¢",
      CenterDot: "·",
      centerdot: "·",
      Cfr: "ℭ",
      cfr: "𝔠",
      CHcy: "Ч",
      chcy: "ч",
      check: "✓",
      checkmark: "✓",
      Chi: "Χ",
      chi: "χ",
      cir: "○",
      circ: "ˆ",
      circeq: "≗",
      circlearrowleft: "↺",
      circlearrowright: "↻",
      circledast: "⊛",
      circledcirc: "⊚",
      circleddash: "⊝",
      CircleDot: "⊙",
      circledR: "®",
      circledS: "Ⓢ",
      CircleMinus: "⊖",
      CirclePlus: "⊕",
      CircleTimes: "⊗",
      cirE: "⧃",
      cire: "≗",
      cirfnint: "⨐",
      cirmid: "⫯",
      cirscir: "⧂",
      ClockwiseContourIntegral: "∲",
      CloseCurlyDoubleQuote: "”",
      CloseCurlyQuote: "’",
      clubs: "♣",
      clubsuit: "♣",
      Colon: "∷",
      colon: ":",
      Colone: "⩴",
      colone: "≔",
      coloneq: "≔",
      comma: ",",
      commat: "@",
      comp: "∁",
      compfn: "∘",
      complement: "∁",
      complexes: "ℂ",
      cong: "≅",
      congdot: "⩭",
      Congruent: "≡",
      Conint: "∯",
      conint: "∮",
      ContourIntegral: "∮",
      Copf: "ℂ",
      copf: "𝕔",
      coprod: "∐",
      Coproduct: "∐",
      COPY: "©",
      copy: "©",
      copysr: "℗",
      CounterClockwiseContourIntegral: "∳",
      crarr: "↵",
      Cross: "⨯",
      cross: "✗",
      Cscr: "𝒞",
      cscr: "𝒸",
      csub: "⫏",
      csube: "⫑",
      csup: "⫐",
      csupe: "⫒",
      ctdot: "⋯",
      cudarrl: "⤸",
      cudarrr: "⤵",
      cuepr: "⋞",
      cuesc: "⋟",
      cularr: "↶",
      cularrp: "⤽",
      Cup: "⋓",
      cup: "∪",
      cupbrcap: "⩈",
      CupCap: "≍",
      cupcap: "⩆",
      cupcup: "⩊",
      cupdot: "⊍",
      cupor: "⩅",
      cups: "∪︀",
      curarr: "↷",
      curarrm: "⤼",
      curlyeqprec: "⋞",
      curlyeqsucc: "⋟",
      curlyvee: "⋎",
      curlywedge: "⋏",
      curren: "¤",
      curvearrowleft: "↶",
      curvearrowright: "↷",
      cuvee: "⋎",
      cuwed: "⋏",
      cwconint: "∲",
      cwint: "∱",
      cylcty: "⌭",
      Dagger: "‡",
      dagger: "†",
      daleth: "ℸ",
      Darr: "↡",
      dArr: "⇓",
      darr: "↓",
      dash: "‐",
      Dashv: "⫤",
      dashv: "⊣",
      dbkarow: "⤏",
      dblac: "˝",
      Dcaron: "Ď",
      dcaron: "ď",
      Dcy: "Д",
      dcy: "д",
      DD: "ⅅ",
      dd: "ⅆ",
      ddagger: "‡",
      ddarr: "⇊",
      DDotrahd: "⤑",
      ddotseq: "⩷",
      deg: "°",
      Del: "∇",
      Delta: "Δ",
      delta: "δ",
      demptyv: "⦱",
      dfisht: "⥿",
      Dfr: "𝔇",
      dfr: "𝔡",
      dHar: "⥥",
      dharl: "⇃",
      dharr: "⇂",
      DiacriticalAcute: "´",
      DiacriticalDot: "˙",
      DiacriticalDoubleAcute: "˝",
      DiacriticalGrave: "`",
      DiacriticalTilde: "˜",
      diam: "⋄",
      Diamond: "⋄",
      diamond: "⋄",
      diamondsuit: "♦",
      diams: "♦",
      die: "¨",
      DifferentialD: "ⅆ",
      digamma: "ϝ",
      disin: "⋲",
      div: "÷",
      divide: "÷",
      divideontimes: "⋇",
      divonx: "⋇",
      DJcy: "Ђ",
      djcy: "ђ",
      dlcorn: "⌞",
      dlcrop: "⌍",
      dollar: "$",
      Dopf: "𝔻",
      dopf: "𝕕",
      Dot: "¨",
      dot: "˙",
      DotDot: "⃜",
      doteq: "≐",
      doteqdot: "≑",
      DotEqual: "≐",
      dotminus: "∸",
      dotplus: "∔",
      dotsquare: "⊡",
      doublebarwedge: "⌆",
      DoubleContourIntegral: "∯",
      DoubleDot: "¨",
      DoubleDownArrow: "⇓",
      DoubleLeftArrow: "⇐",
      DoubleLeftRightArrow: "⇔",
      DoubleLeftTee: "⫤",
      DoubleLongLeftArrow: "⟸",
      DoubleLongLeftRightArrow: "⟺",
      DoubleLongRightArrow: "⟹",
      DoubleRightArrow: "⇒",
      DoubleRightTee: "⊨",
      DoubleUpArrow: "⇑",
      DoubleUpDownArrow: "⇕",
      DoubleVerticalBar: "∥",
      DownArrow: "↓",
      Downarrow: "⇓",
      downarrow: "↓",
      DownArrowBar: "⤓",
      DownArrowUpArrow: "⇵",
      DownBreve: "̑",
      downdownarrows: "⇊",
      downharpoonleft: "⇃",
      downharpoonright: "⇂",
      DownLeftRightVector: "⥐",
      DownLeftTeeVector: "⥞",
      DownLeftVector: "↽",
      DownLeftVectorBar: "⥖",
      DownRightTeeVector: "⥟",
      DownRightVector: "⇁",
      DownRightVectorBar: "⥗",
      DownTee: "⊤",
      DownTeeArrow: "↧",
      drbkarow: "⤐",
      drcorn: "⌟",
      drcrop: "⌌",
      Dscr: "𝒟",
      dscr: "𝒹",
      DScy: "Ѕ",
      dscy: "ѕ",
      dsol: "⧶",
      Dstrok: "Đ",
      dstrok: "đ",
      dtdot: "⋱",
      dtri: "▿",
      dtrif: "▾",
      duarr: "⇵",
      duhar: "⥯",
      dwangle: "⦦",
      DZcy: "Џ",
      dzcy: "џ",
      dzigrarr: "⟿",
      Eacute: "É",
      eacute: "é",
      easter: "⩮",
      Ecaron: "Ě",
      ecaron: "ě",
      ecir: "≖",
      Ecirc: "Ê",
      ecirc: "ê",
      ecolon: "≕",
      Ecy: "Э",
      ecy: "э",
      eDDot: "⩷",
      Edot: "Ė",
      eDot: "≑",
      edot: "ė",
      ee: "ⅇ",
      efDot: "≒",
      Efr: "𝔈",
      efr: "𝔢",
      eg: "⪚",
      Egrave: "È",
      egrave: "è",
      egs: "⪖",
      egsdot: "⪘",
      el: "⪙",
      Element: "∈",
      elinters: "⏧",
      ell: "ℓ",
      els: "⪕",
      elsdot: "⪗",
      Emacr: "Ē",
      emacr: "ē",
      empty: "∅",
      emptyset: "∅",
      EmptySmallSquare: "◻",
      emptyv: "∅",
      EmptyVerySmallSquare: "▫",
      emsp: " ",
      emsp13: " ",
      emsp14: " ",
      ENG: "Ŋ",
      eng: "ŋ",
      ensp: " ",
      Eogon: "Ę",
      eogon: "ę",
      Eopf: "𝔼",
      eopf: "𝕖",
      epar: "⋕",
      eparsl: "⧣",
      eplus: "⩱",
      epsi: "ε",
      Epsilon: "Ε",
      epsilon: "ε",
      epsiv: "ϵ",
      eqcirc: "≖",
      eqcolon: "≕",
      eqsim: "≂",
      eqslantgtr: "⪖",
      eqslantless: "⪕",
      Equal: "⩵",
      equals: "=",
      EqualTilde: "≂",
      equest: "≟",
      Equilibrium: "⇌",
      equiv: "≡",
      equivDD: "⩸",
      eqvparsl: "⧥",
      erarr: "⥱",
      erDot: "≓",
      Escr: "ℰ",
      escr: "ℯ",
      esdot: "≐",
      Esim: "⩳",
      esim: "≂",
      Eta: "Η",
      eta: "η",
      ETH: "Ð",
      eth: "ð",
      Euml: "Ë",
      euml: "ë",
      euro: "€",
      excl: "!",
      exist: "∃",
      Exists: "∃",
      expectation: "ℰ",
      ExponentialE: "ⅇ",
      exponentiale: "ⅇ",
      fallingdotseq: "≒",
      Fcy: "Ф",
      fcy: "ф",
      female: "♀",
      ffilig: "ﬃ",
      fflig: "ﬀ",
      ffllig: "ﬄ",
      Ffr: "𝔉",
      ffr: "𝔣",
      filig: "ﬁ",
      FilledSmallSquare: "◼",
      FilledVerySmallSquare: "▪",
      fjlig: "fj",
      flat: "♭",
      fllig: "ﬂ",
      fltns: "▱",
      fnof: "ƒ",
      Fopf: "𝔽",
      fopf: "𝕗",
      ForAll: "∀",
      forall: "∀",
      fork: "⋔",
      forkv: "⫙",
      Fouriertrf: "ℱ",
      fpartint: "⨍",
      frac12: "½",
      frac13: "⅓",
      frac14: "¼",
      frac15: "⅕",
      frac16: "⅙",
      frac18: "⅛",
      frac23: "⅔",
      frac25: "⅖",
      frac34: "¾",
      frac35: "⅗",
      frac38: "⅜",
      frac45: "⅘",
      frac56: "⅚",
      frac58: "⅝",
      frac78: "⅞",
      frasl: "⁄",
      frown: "⌢",
      Fscr: "ℱ",
      fscr: "𝒻",
      gacute: "ǵ",
      Gamma: "Γ",
      gamma: "γ",
      Gammad: "Ϝ",
      gammad: "ϝ",
      gap: "⪆",
      Gbreve: "Ğ",
      gbreve: "ğ",
      Gcedil: "Ģ",
      Gcirc: "Ĝ",
      gcirc: "ĝ",
      Gcy: "Г",
      gcy: "г",
      Gdot: "Ġ",
      gdot: "ġ",
      gE: "≧",
      ge: "≥",
      gEl: "⪌",
      gel: "⋛",
      geq: "≥",
      geqq: "≧",
      geqslant: "⩾",
      ges: "⩾",
      gescc: "⪩",
      gesdot: "⪀",
      gesdoto: "⪂",
      gesdotol: "⪄",
      gesl: "⋛︀",
      gesles: "⪔",
      Gfr: "𝔊",
      gfr: "𝔤",
      Gg: "⋙",
      gg: "≫",
      ggg: "⋙",
      gimel: "ℷ",
      GJcy: "Ѓ",
      gjcy: "ѓ",
      gl: "≷",
      gla: "⪥",
      glE: "⪒",
      glj: "⪤",
      gnap: "⪊",
      gnapprox: "⪊",
      gnE: "≩",
      gne: "⪈",
      gneq: "⪈",
      gneqq: "≩",
      gnsim: "⋧",
      Gopf: "𝔾",
      gopf: "𝕘",
      grave: "`",
      GreaterEqual: "≥",
      GreaterEqualLess: "⋛",
      GreaterFullEqual: "≧",
      GreaterGreater: "⪢",
      GreaterLess: "≷",
      GreaterSlantEqual: "⩾",
      GreaterTilde: "≳",
      Gscr: "𝒢",
      gscr: "ℊ",
      gsim: "≳",
      gsime: "⪎",
      gsiml: "⪐",
      Gt: "≫",
      GT: ">",
      gt: ">",
      gtcc: "⪧",
      gtcir: "⩺",
      gtdot: "⋗",
      gtlPar: "⦕",
      gtquest: "⩼",
      gtrapprox: "⪆",
      gtrarr: "⥸",
      gtrdot: "⋗",
      gtreqless: "⋛",
      gtreqqless: "⪌",
      gtrless: "≷",
      gtrsim: "≳",
      gvertneqq: "≩︀",
      gvnE: "≩︀",
      Hacek: "ˇ",
      hairsp: " ",
      half: "½",
      hamilt: "ℋ",
      HARDcy: "Ъ",
      hardcy: "ъ",
      hArr: "⇔",
      harr: "↔",
      harrcir: "⥈",
      harrw: "↭",
      Hat: "^",
      hbar: "ℏ",
      Hcirc: "Ĥ",
      hcirc: "ĥ",
      hearts: "♥",
      heartsuit: "♥",
      hellip: "…",
      hercon: "⊹",
      Hfr: "ℌ",
      hfr: "𝔥",
      HilbertSpace: "ℋ",
      hksearow: "⤥",
      hkswarow: "⤦",
      hoarr: "⇿",
      homtht: "∻",
      hookleftarrow: "↩",
      hookrightarrow: "↪",
      Hopf: "ℍ",
      hopf: "𝕙",
      horbar: "―",
      HorizontalLine: "─",
      Hscr: "ℋ",
      hscr: "𝒽",
      hslash: "ℏ",
      Hstrok: "Ħ",
      hstrok: "ħ",
      HumpDownHump: "≎",
      HumpEqual: "≏",
      hybull: "⁃",
      hyphen: "‐",
      Iacute: "Í",
      iacute: "í",
      ic: "⁣",
      Icirc: "Î",
      icirc: "î",
      Icy: "И",
      icy: "и",
      Idot: "İ",
      IEcy: "Е",
      iecy: "е",
      iexcl: "¡",
      iff: "⇔",
      Ifr: "ℑ",
      ifr: "𝔦",
      Igrave: "Ì",
      igrave: "ì",
      ii: "ⅈ",
      iiiint: "⨌",
      iiint: "∭",
      iinfin: "⧜",
      iiota: "℩",
      IJlig: "Ĳ",
      ijlig: "ĳ",
      Im: "ℑ",
      Imacr: "Ī",
      imacr: "ī",
      image: "ℑ",
      ImaginaryI: "ⅈ",
      imagline: "ℐ",
      imagpart: "ℑ",
      imath: "ı",
      imof: "⊷",
      imped: "Ƶ",
      Implies: "⇒",
      in: "∈",
      incare: "℅",
      infin: "∞",
      infintie: "⧝",
      inodot: "ı",
      Int: "∬",
      int: "∫",
      intcal: "⊺",
      integers: "ℤ",
      Integral: "∫",
      intercal: "⊺",
      Intersection: "⋂",
      intlarhk: "⨗",
      intprod: "⨼",
      InvisibleComma: "⁣",
      InvisibleTimes: "⁢",
      IOcy: "Ё",
      iocy: "ё",
      Iogon: "Į",
      iogon: "į",
      Iopf: "𝕀",
      iopf: "𝕚",
      Iota: "Ι",
      iota: "ι",
      iprod: "⨼",
      iquest: "¿",
      Iscr: "ℐ",
      iscr: "𝒾",
      isin: "∈",
      isindot: "⋵",
      isinE: "⋹",
      isins: "⋴",
      isinsv: "⋳",
      isinv: "∈",
      it: "⁢",
      Itilde: "Ĩ",
      itilde: "ĩ",
      Iukcy: "І",
      iukcy: "і",
      Iuml: "Ï",
      iuml: "ï",
      Jcirc: "Ĵ",
      jcirc: "ĵ",
      Jcy: "Й",
      jcy: "й",
      Jfr: "𝔍",
      jfr: "𝔧",
      jmath: "ȷ",
      Jopf: "𝕁",
      jopf: "𝕛",
      Jscr: "𝒥",
      jscr: "𝒿",
      Jsercy: "Ј",
      jsercy: "ј",
      Jukcy: "Є",
      jukcy: "є",
      Kappa: "Κ",
      kappa: "κ",
      kappav: "ϰ",
      Kcedil: "Ķ",
      kcedil: "ķ",
      Kcy: "К",
      kcy: "к",
      Kfr: "𝔎",
      kfr: "𝔨",
      kgreen: "ĸ",
      KHcy: "Х",
      khcy: "х",
      KJcy: "Ќ",
      kjcy: "ќ",
      Kopf: "𝕂",
      kopf: "𝕜",
      Kscr: "𝒦",
      kscr: "𝓀",
      lAarr: "⇚",
      Lacute: "Ĺ",
      lacute: "ĺ",
      laemptyv: "⦴",
      lagran: "ℒ",
      Lambda: "Λ",
      lambda: "λ",
      Lang: "⟪",
      lang: "⟨",
      langd: "⦑",
      langle: "⟨",
      lap: "⪅",
      Laplacetrf: "ℒ",
      laquo: "«",
      Larr: "↞",
      lArr: "⇐",
      larr: "←",
      larrb: "⇤",
      larrbfs: "⤟",
      larrfs: "⤝",
      larrhk: "↩",
      larrlp: "↫",
      larrpl: "⤹",
      larrsim: "⥳",
      larrtl: "↢",
      lat: "⪫",
      lAtail: "⤛",
      latail: "⤙",
      late: "⪭",
      lates: "⪭︀",
      lBarr: "⤎",
      lbarr: "⤌",
      lbbrk: "❲",
      lbrace: "{",
      lbrack: "[",
      lbrke: "⦋",
      lbrksld: "⦏",
      lbrkslu: "⦍",
      Lcaron: "Ľ",
      lcaron: "ľ",
      Lcedil: "Ļ",
      lcedil: "ļ",
      lceil: "⌈",
      lcub: "{",
      Lcy: "Л",
      lcy: "л",
      ldca: "⤶",
      ldquo: "“",
      ldquor: "„",
      ldrdhar: "⥧",
      ldrushar: "⥋",
      ldsh: "↲",
      lE: "≦",
      le: "≤",
      LeftAngleBracket: "⟨",
      LeftArrow: "←",
      Leftarrow: "⇐",
      leftarrow: "←",
      LeftArrowBar: "⇤",
      LeftArrowRightArrow: "⇆",
      leftarrowtail: "↢",
      LeftCeiling: "⌈",
      LeftDoubleBracket: "⟦",
      LeftDownTeeVector: "⥡",
      LeftDownVector: "⇃",
      LeftDownVectorBar: "⥙",
      LeftFloor: "⌊",
      leftharpoondown: "↽",
      leftharpoonup: "↼",
      leftleftarrows: "⇇",
      LeftRightArrow: "↔",
      Leftrightarrow: "⇔",
      leftrightarrow: "↔",
      leftrightarrows: "⇆",
      leftrightharpoons: "⇋",
      leftrightsquigarrow: "↭",
      LeftRightVector: "⥎",
      LeftTee: "⊣",
      LeftTeeArrow: "↤",
      LeftTeeVector: "⥚",
      leftthreetimes: "⋋",
      LeftTriangle: "⊲",
      LeftTriangleBar: "⧏",
      LeftTriangleEqual: "⊴",
      LeftUpDownVector: "⥑",
      LeftUpTeeVector: "⥠",
      LeftUpVector: "↿",
      LeftUpVectorBar: "⥘",
      LeftVector: "↼",
      LeftVectorBar: "⥒",
      lEg: "⪋",
      leg: "⋚",
      leq: "≤",
      leqq: "≦",
      leqslant: "⩽",
      les: "⩽",
      lescc: "⪨",
      lesdot: "⩿",
      lesdoto: "⪁",
      lesdotor: "⪃",
      lesg: "⋚︀",
      lesges: "⪓",
      lessapprox: "⪅",
      lessdot: "⋖",
      lesseqgtr: "⋚",
      lesseqqgtr: "⪋",
      LessEqualGreater: "⋚",
      LessFullEqual: "≦",
      LessGreater: "≶",
      lessgtr: "≶",
      LessLess: "⪡",
      lesssim: "≲",
      LessSlantEqual: "⩽",
      LessTilde: "≲",
      lfisht: "⥼",
      lfloor: "⌊",
      Lfr: "𝔏",
      lfr: "𝔩",
      lg: "≶",
      lgE: "⪑",
      lHar: "⥢",
      lhard: "↽",
      lharu: "↼",
      lharul: "⥪",
      lhblk: "▄",
      LJcy: "Љ",
      ljcy: "љ",
      Ll: "⋘",
      ll: "≪",
      llarr: "⇇",
      llcorner: "⌞",
      Lleftarrow: "⇚",
      llhard: "⥫",
      lltri: "◺",
      Lmidot: "Ŀ",
      lmidot: "ŀ",
      lmoust: "⎰",
      lmoustache: "⎰",
      lnap: "⪉",
      lnapprox: "⪉",
      lnE: "≨",
      lne: "⪇",
      lneq: "⪇",
      lneqq: "≨",
      lnsim: "⋦",
      loang: "⟬",
      loarr: "⇽",
      lobrk: "⟦",
      LongLeftArrow: "⟵",
      Longleftarrow: "⟸",
      longleftarrow: "⟵",
      LongLeftRightArrow: "⟷",
      Longleftrightarrow: "⟺",
      longleftrightarrow: "⟷",
      longmapsto: "⟼",
      LongRightArrow: "⟶",
      Longrightarrow: "⟹",
      longrightarrow: "⟶",
      looparrowleft: "↫",
      looparrowright: "↬",
      lopar: "⦅",
      Lopf: "𝕃",
      lopf: "𝕝",
      loplus: "⨭",
      lotimes: "⨴",
      lowast: "∗",
      lowbar: "_",
      LowerLeftArrow: "↙",
      LowerRightArrow: "↘",
      loz: "◊",
      lozenge: "◊",
      lozf: "⧫",
      lpar: "(",
      lparlt: "⦓",
      lrarr: "⇆",
      lrcorner: "⌟",
      lrhar: "⇋",
      lrhard: "⥭",
      lrm: "‎",
      lrtri: "⊿",
      lsaquo: "‹",
      Lscr: "ℒ",
      lscr: "𝓁",
      Lsh: "↰",
      lsh: "↰",
      lsim: "≲",
      lsime: "⪍",
      lsimg: "⪏",
      lsqb: "[",
      lsquo: "‘",
      lsquor: "‚",
      Lstrok: "Ł",
      lstrok: "ł",
      Lt: "≪",
      LT: "<",
      lt: "<",
      ltcc: "⪦",
      ltcir: "⩹",
      ltdot: "⋖",
      lthree: "⋋",
      ltimes: "⋉",
      ltlarr: "⥶",
      ltquest: "⩻",
      ltri: "◃",
      ltrie: "⊴",
      ltrif: "◂",
      ltrPar: "⦖",
      lurdshar: "⥊",
      luruhar: "⥦",
      lvertneqq: "≨︀",
      lvnE: "≨︀",
      macr: "¯",
      male: "♂",
      malt: "✠",
      maltese: "✠",
      Map: "⤅",
      map: "↦",
      mapsto: "↦",
      mapstodown: "↧",
      mapstoleft: "↤",
      mapstoup: "↥",
      marker: "▮",
      mcomma: "⨩",
      Mcy: "М",
      mcy: "м",
      mdash: "—",
      mDDot: "∺",
      measuredangle: "∡",
      MediumSpace: " ",
      Mellintrf: "ℳ",
      Mfr: "𝔐",
      mfr: "𝔪",
      mho: "℧",
      micro: "µ",
      mid: "∣",
      midast: "*",
      midcir: "⫰",
      middot: "·",
      minus: "−",
      minusb: "⊟",
      minusd: "∸",
      minusdu: "⨪",
      MinusPlus: "∓",
      mlcp: "⫛",
      mldr: "…",
      mnplus: "∓",
      models: "⊧",
      Mopf: "𝕄",
      mopf: "𝕞",
      mp: "∓",
      Mscr: "ℳ",
      mscr: "𝓂",
      mstpos: "∾",
      Mu: "Μ",
      mu: "μ",
      multimap: "⊸",
      mumap: "⊸",
      nabla: "∇",
      Nacute: "Ń",
      nacute: "ń",
      nang: "∠⃒",
      nap: "≉",
      napE: "⩰̸",
      napid: "≋̸",
      napos: "ŉ",
      napprox: "≉",
      natur: "♮",
      natural: "♮",
      naturals: "ℕ",
      nbsp: " ",
      nbump: "≎̸",
      nbumpe: "≏̸",
      ncap: "⩃",
      Ncaron: "Ň",
      ncaron: "ň",
      Ncedil: "Ņ",
      ncedil: "ņ",
      ncong: "≇",
      ncongdot: "⩭̸",
      ncup: "⩂",
      Ncy: "Н",
      ncy: "н",
      ndash: "–",
      ne: "≠",
      nearhk: "⤤",
      neArr: "⇗",
      nearr: "↗",
      nearrow: "↗",
      nedot: "≐̸",
      NegativeMediumSpace: "​",
      NegativeThickSpace: "​",
      NegativeThinSpace: "​",
      NegativeVeryThinSpace: "​",
      nequiv: "≢",
      nesear: "⤨",
      nesim: "≂̸",
      NestedGreaterGreater: "≫",
      NestedLessLess: "≪",
      NewLine: `
`,
      nexist: "∄",
      nexists: "∄",
      Nfr: "𝔑",
      nfr: "𝔫",
      ngE: "≧̸",
      nge: "≱",
      ngeq: "≱",
      ngeqq: "≧̸",
      ngeqslant: "⩾̸",
      nges: "⩾̸",
      nGg: "⋙̸",
      ngsim: "≵",
      nGt: "≫⃒",
      ngt: "≯",
      ngtr: "≯",
      nGtv: "≫̸",
      nhArr: "⇎",
      nharr: "↮",
      nhpar: "⫲",
      ni: "∋",
      nis: "⋼",
      nisd: "⋺",
      niv: "∋",
      NJcy: "Њ",
      njcy: "њ",
      nlArr: "⇍",
      nlarr: "↚",
      nldr: "‥",
      nlE: "≦̸",
      nle: "≰",
      nLeftarrow: "⇍",
      nleftarrow: "↚",
      nLeftrightarrow: "⇎",
      nleftrightarrow: "↮",
      nleq: "≰",
      nleqq: "≦̸",
      nleqslant: "⩽̸",
      nles: "⩽̸",
      nless: "≮",
      nLl: "⋘̸",
      nlsim: "≴",
      nLt: "≪⃒",
      nlt: "≮",
      nltri: "⋪",
      nltrie: "⋬",
      nLtv: "≪̸",
      nmid: "∤",
      NoBreak: "⁠",
      NonBreakingSpace: " ",
      Nopf: "ℕ",
      nopf: "𝕟",
      Not: "⫬",
      not: "¬",
      NotCongruent: "≢",
      NotCupCap: "≭",
      NotDoubleVerticalBar: "∦",
      NotElement: "∉",
      NotEqual: "≠",
      NotEqualTilde: "≂̸",
      NotExists: "∄",
      NotGreater: "≯",
      NotGreaterEqual: "≱",
      NotGreaterFullEqual: "≧̸",
      NotGreaterGreater: "≫̸",
      NotGreaterLess: "≹",
      NotGreaterSlantEqual: "⩾̸",
      NotGreaterTilde: "≵",
      NotHumpDownHump: "≎̸",
      NotHumpEqual: "≏̸",
      notin: "∉",
      notindot: "⋵̸",
      notinE: "⋹̸",
      notinva: "∉",
      notinvb: "⋷",
      notinvc: "⋶",
      NotLeftTriangle: "⋪",
      NotLeftTriangleBar: "⧏̸",
      NotLeftTriangleEqual: "⋬",
      NotLess: "≮",
      NotLessEqual: "≰",
      NotLessGreater: "≸",
      NotLessLess: "≪̸",
      NotLessSlantEqual: "⩽̸",
      NotLessTilde: "≴",
      NotNestedGreaterGreater: "⪢̸",
      NotNestedLessLess: "⪡̸",
      notni: "∌",
      notniva: "∌",
      notnivb: "⋾",
      notnivc: "⋽",
      NotPrecedes: "⊀",
      NotPrecedesEqual: "⪯̸",
      NotPrecedesSlantEqual: "⋠",
      NotReverseElement: "∌",
      NotRightTriangle: "⋫",
      NotRightTriangleBar: "⧐̸",
      NotRightTriangleEqual: "⋭",
      NotSquareSubset: "⊏̸",
      NotSquareSubsetEqual: "⋢",
      NotSquareSuperset: "⊐̸",
      NotSquareSupersetEqual: "⋣",
      NotSubset: "⊂⃒",
      NotSubsetEqual: "⊈",
      NotSucceeds: "⊁",
      NotSucceedsEqual: "⪰̸",
      NotSucceedsSlantEqual: "⋡",
      NotSucceedsTilde: "≿̸",
      NotSuperset: "⊃⃒",
      NotSupersetEqual: "⊉",
      NotTilde: "≁",
      NotTildeEqual: "≄",
      NotTildeFullEqual: "≇",
      NotTildeTilde: "≉",
      NotVerticalBar: "∤",
      npar: "∦",
      nparallel: "∦",
      nparsl: "⫽⃥",
      npart: "∂̸",
      npolint: "⨔",
      npr: "⊀",
      nprcue: "⋠",
      npre: "⪯̸",
      nprec: "⊀",
      npreceq: "⪯̸",
      nrArr: "⇏",
      nrarr: "↛",
      nrarrc: "⤳̸",
      nrarrw: "↝̸",
      nRightarrow: "⇏",
      nrightarrow: "↛",
      nrtri: "⋫",
      nrtrie: "⋭",
      nsc: "⊁",
      nsccue: "⋡",
      nsce: "⪰̸",
      Nscr: "𝒩",
      nscr: "𝓃",
      nshortmid: "∤",
      nshortparallel: "∦",
      nsim: "≁",
      nsime: "≄",
      nsimeq: "≄",
      nsmid: "∤",
      nspar: "∦",
      nsqsube: "⋢",
      nsqsupe: "⋣",
      nsub: "⊄",
      nsubE: "⫅̸",
      nsube: "⊈",
      nsubset: "⊂⃒",
      nsubseteq: "⊈",
      nsubseteqq: "⫅̸",
      nsucc: "⊁",
      nsucceq: "⪰̸",
      nsup: "⊅",
      nsupE: "⫆̸",
      nsupe: "⊉",
      nsupset: "⊃⃒",
      nsupseteq: "⊉",
      nsupseteqq: "⫆̸",
      ntgl: "≹",
      Ntilde: "Ñ",
      ntilde: "ñ",
      ntlg: "≸",
      ntriangleleft: "⋪",
      ntrianglelefteq: "⋬",
      ntriangleright: "⋫",
      ntrianglerighteq: "⋭",
      Nu: "Ν",
      nu: "ν",
      num: "#",
      numero: "№",
      numsp: " ",
      nvap: "≍⃒",
      nVDash: "⊯",
      nVdash: "⊮",
      nvDash: "⊭",
      nvdash: "⊬",
      nvge: "≥⃒",
      nvgt: ">⃒",
      nvHarr: "⤄",
      nvinfin: "⧞",
      nvlArr: "⤂",
      nvle: "≤⃒",
      nvlt: "<⃒",
      nvltrie: "⊴⃒",
      nvrArr: "⤃",
      nvrtrie: "⊵⃒",
      nvsim: "∼⃒",
      nwarhk: "⤣",
      nwArr: "⇖",
      nwarr: "↖",
      nwarrow: "↖",
      nwnear: "⤧",
      Oacute: "Ó",
      oacute: "ó",
      oast: "⊛",
      ocir: "⊚",
      Ocirc: "Ô",
      ocirc: "ô",
      Ocy: "О",
      ocy: "о",
      odash: "⊝",
      Odblac: "Ő",
      odblac: "ő",
      odiv: "⨸",
      odot: "⊙",
      odsold: "⦼",
      OElig: "Œ",
      oelig: "œ",
      ofcir: "⦿",
      Ofr: "𝔒",
      ofr: "𝔬",
      ogon: "˛",
      Ograve: "Ò",
      ograve: "ò",
      ogt: "⧁",
      ohbar: "⦵",
      ohm: "Ω",
      oint: "∮",
      olarr: "↺",
      olcir: "⦾",
      olcross: "⦻",
      oline: "‾",
      olt: "⧀",
      Omacr: "Ō",
      omacr: "ō",
      Omega: "Ω",
      omega: "ω",
      Omicron: "Ο",
      omicron: "ο",
      omid: "⦶",
      ominus: "⊖",
      Oopf: "𝕆",
      oopf: "𝕠",
      opar: "⦷",
      OpenCurlyDoubleQuote: "“",
      OpenCurlyQuote: "‘",
      operp: "⦹",
      oplus: "⊕",
      Or: "⩔",
      or: "∨",
      orarr: "↻",
      ord: "⩝",
      order: "ℴ",
      orderof: "ℴ",
      ordf: "ª",
      ordm: "º",
      origof: "⊶",
      oror: "⩖",
      orslope: "⩗",
      orv: "⩛",
      oS: "Ⓢ",
      Oscr: "𝒪",
      oscr: "ℴ",
      Oslash: "Ø",
      oslash: "ø",
      osol: "⊘",
      Otilde: "Õ",
      otilde: "õ",
      Otimes: "⨷",
      otimes: "⊗",
      otimesas: "⨶",
      Ouml: "Ö",
      ouml: "ö",
      ovbar: "⌽",
      OverBar: "‾",
      OverBrace: "⏞",
      OverBracket: "⎴",
      OverParenthesis: "⏜",
      par: "∥",
      para: "¶",
      parallel: "∥",
      parsim: "⫳",
      parsl: "⫽",
      part: "∂",
      PartialD: "∂",
      Pcy: "П",
      pcy: "п",
      percnt: "%",
      period: ".",
      permil: "‰",
      perp: "⊥",
      pertenk: "‱",
      Pfr: "𝔓",
      pfr: "𝔭",
      Phi: "Φ",
      phi: "φ",
      phiv: "ϕ",
      phmmat: "ℳ",
      phone: "☎",
      Pi: "Π",
      pi: "π",
      pitchfork: "⋔",
      piv: "ϖ",
      planck: "ℏ",
      planckh: "ℎ",
      plankv: "ℏ",
      plus: "+",
      plusacir: "⨣",
      plusb: "⊞",
      pluscir: "⨢",
      plusdo: "∔",
      plusdu: "⨥",
      pluse: "⩲",
      PlusMinus: "±",
      plusmn: "±",
      plussim: "⨦",
      plustwo: "⨧",
      pm: "±",
      Poincareplane: "ℌ",
      pointint: "⨕",
      Popf: "ℙ",
      popf: "𝕡",
      pound: "£",
      Pr: "⪻",
      pr: "≺",
      prap: "⪷",
      prcue: "≼",
      prE: "⪳",
      pre: "⪯",
      prec: "≺",
      precapprox: "⪷",
      preccurlyeq: "≼",
      Precedes: "≺",
      PrecedesEqual: "⪯",
      PrecedesSlantEqual: "≼",
      PrecedesTilde: "≾",
      preceq: "⪯",
      precnapprox: "⪹",
      precneqq: "⪵",
      precnsim: "⋨",
      precsim: "≾",
      Prime: "″",
      prime: "′",
      primes: "ℙ",
      prnap: "⪹",
      prnE: "⪵",
      prnsim: "⋨",
      prod: "∏",
      Product: "∏",
      profalar: "⌮",
      profline: "⌒",
      profsurf: "⌓",
      prop: "∝",
      Proportion: "∷",
      Proportional: "∝",
      propto: "∝",
      prsim: "≾",
      prurel: "⊰",
      Pscr: "𝒫",
      pscr: "𝓅",
      Psi: "Ψ",
      psi: "ψ",
      puncsp: " ",
      Qfr: "𝔔",
      qfr: "𝔮",
      qint: "⨌",
      Qopf: "ℚ",
      qopf: "𝕢",
      qprime: "⁗",
      Qscr: "𝒬",
      qscr: "𝓆",
      quaternions: "ℍ",
      quatint: "⨖",
      quest: "?",
      questeq: "≟",
      QUOT: '"',
      quot: '"',
      rAarr: "⇛",
      race: "∽̱",
      Racute: "Ŕ",
      racute: "ŕ",
      radic: "√",
      raemptyv: "⦳",
      Rang: "⟫",
      rang: "⟩",
      rangd: "⦒",
      range: "⦥",
      rangle: "⟩",
      raquo: "»",
      Rarr: "↠",
      rArr: "⇒",
      rarr: "→",
      rarrap: "⥵",
      rarrb: "⇥",
      rarrbfs: "⤠",
      rarrc: "⤳",
      rarrfs: "⤞",
      rarrhk: "↪",
      rarrlp: "↬",
      rarrpl: "⥅",
      rarrsim: "⥴",
      Rarrtl: "⤖",
      rarrtl: "↣",
      rarrw: "↝",
      rAtail: "⤜",
      ratail: "⤚",
      ratio: "∶",
      rationals: "ℚ",
      RBarr: "⤐",
      rBarr: "⤏",
      rbarr: "⤍",
      rbbrk: "❳",
      rbrace: "}",
      rbrack: "]",
      rbrke: "⦌",
      rbrksld: "⦎",
      rbrkslu: "⦐",
      Rcaron: "Ř",
      rcaron: "ř",
      Rcedil: "Ŗ",
      rcedil: "ŗ",
      rceil: "⌉",
      rcub: "}",
      Rcy: "Р",
      rcy: "р",
      rdca: "⤷",
      rdldhar: "⥩",
      rdquo: "”",
      rdquor: "”",
      rdsh: "↳",
      Re: "ℜ",
      real: "ℜ",
      realine: "ℛ",
      realpart: "ℜ",
      reals: "ℝ",
      rect: "▭",
      REG: "®",
      reg: "®",
      ReverseElement: "∋",
      ReverseEquilibrium: "⇋",
      ReverseUpEquilibrium: "⥯",
      rfisht: "⥽",
      rfloor: "⌋",
      Rfr: "ℜ",
      rfr: "𝔯",
      rHar: "⥤",
      rhard: "⇁",
      rharu: "⇀",
      rharul: "⥬",
      Rho: "Ρ",
      rho: "ρ",
      rhov: "ϱ",
      RightAngleBracket: "⟩",
      RightArrow: "→",
      Rightarrow: "⇒",
      rightarrow: "→",
      RightArrowBar: "⇥",
      RightArrowLeftArrow: "⇄",
      rightarrowtail: "↣",
      RightCeiling: "⌉",
      RightDoubleBracket: "⟧",
      RightDownTeeVector: "⥝",
      RightDownVector: "⇂",
      RightDownVectorBar: "⥕",
      RightFloor: "⌋",
      rightharpoondown: "⇁",
      rightharpoonup: "⇀",
      rightleftarrows: "⇄",
      rightleftharpoons: "⇌",
      rightrightarrows: "⇉",
      rightsquigarrow: "↝",
      RightTee: "⊢",
      RightTeeArrow: "↦",
      RightTeeVector: "⥛",
      rightthreetimes: "⋌",
      RightTriangle: "⊳",
      RightTriangleBar: "⧐",
      RightTriangleEqual: "⊵",
      RightUpDownVector: "⥏",
      RightUpTeeVector: "⥜",
      RightUpVector: "↾",
      RightUpVectorBar: "⥔",
      RightVector: "⇀",
      RightVectorBar: "⥓",
      ring: "˚",
      risingdotseq: "≓",
      rlarr: "⇄",
      rlhar: "⇌",
      rlm: "‏",
      rmoust: "⎱",
      rmoustache: "⎱",
      rnmid: "⫮",
      roang: "⟭",
      roarr: "⇾",
      robrk: "⟧",
      ropar: "⦆",
      Ropf: "ℝ",
      ropf: "𝕣",
      roplus: "⨮",
      rotimes: "⨵",
      RoundImplies: "⥰",
      rpar: ")",
      rpargt: "⦔",
      rppolint: "⨒",
      rrarr: "⇉",
      Rrightarrow: "⇛",
      rsaquo: "›",
      Rscr: "ℛ",
      rscr: "𝓇",
      Rsh: "↱",
      rsh: "↱",
      rsqb: "]",
      rsquo: "’",
      rsquor: "’",
      rthree: "⋌",
      rtimes: "⋊",
      rtri: "▹",
      rtrie: "⊵",
      rtrif: "▸",
      rtriltri: "⧎",
      RuleDelayed: "⧴",
      ruluhar: "⥨",
      rx: "℞",
      Sacute: "Ś",
      sacute: "ś",
      sbquo: "‚",
      Sc: "⪼",
      sc: "≻",
      scap: "⪸",
      Scaron: "Š",
      scaron: "š",
      sccue: "≽",
      scE: "⪴",
      sce: "⪰",
      Scedil: "Ş",
      scedil: "ş",
      Scirc: "Ŝ",
      scirc: "ŝ",
      scnap: "⪺",
      scnE: "⪶",
      scnsim: "⋩",
      scpolint: "⨓",
      scsim: "≿",
      Scy: "С",
      scy: "с",
      sdot: "⋅",
      sdotb: "⊡",
      sdote: "⩦",
      searhk: "⤥",
      seArr: "⇘",
      searr: "↘",
      searrow: "↘",
      sect: "§",
      semi: ";",
      seswar: "⤩",
      setminus: "∖",
      setmn: "∖",
      sext: "✶",
      Sfr: "𝔖",
      sfr: "𝔰",
      sfrown: "⌢",
      sharp: "♯",
      SHCHcy: "Щ",
      shchcy: "щ",
      SHcy: "Ш",
      shcy: "ш",
      ShortDownArrow: "↓",
      ShortLeftArrow: "←",
      shortmid: "∣",
      shortparallel: "∥",
      ShortRightArrow: "→",
      ShortUpArrow: "↑",
      shy: "­",
      Sigma: "Σ",
      sigma: "σ",
      sigmaf: "ς",
      sigmav: "ς",
      sim: "∼",
      simdot: "⩪",
      sime: "≃",
      simeq: "≃",
      simg: "⪞",
      simgE: "⪠",
      siml: "⪝",
      simlE: "⪟",
      simne: "≆",
      simplus: "⨤",
      simrarr: "⥲",
      slarr: "←",
      SmallCircle: "∘",
      smallsetminus: "∖",
      smashp: "⨳",
      smeparsl: "⧤",
      smid: "∣",
      smile: "⌣",
      smt: "⪪",
      smte: "⪬",
      smtes: "⪬︀",
      SOFTcy: "Ь",
      softcy: "ь",
      sol: "/",
      solb: "⧄",
      solbar: "⌿",
      Sopf: "𝕊",
      sopf: "𝕤",
      spades: "♠",
      spadesuit: "♠",
      spar: "∥",
      sqcap: "⊓",
      sqcaps: "⊓︀",
      sqcup: "⊔",
      sqcups: "⊔︀",
      Sqrt: "√",
      sqsub: "⊏",
      sqsube: "⊑",
      sqsubset: "⊏",
      sqsubseteq: "⊑",
      sqsup: "⊐",
      sqsupe: "⊒",
      sqsupset: "⊐",
      sqsupseteq: "⊒",
      squ: "□",
      Square: "□",
      square: "□",
      SquareIntersection: "⊓",
      SquareSubset: "⊏",
      SquareSubsetEqual: "⊑",
      SquareSuperset: "⊐",
      SquareSupersetEqual: "⊒",
      SquareUnion: "⊔",
      squarf: "▪",
      squf: "▪",
      srarr: "→",
      Sscr: "𝒮",
      sscr: "𝓈",
      ssetmn: "∖",
      ssmile: "⌣",
      sstarf: "⋆",
      Star: "⋆",
      star: "☆",
      starf: "★",
      straightepsilon: "ϵ",
      straightphi: "ϕ",
      strns: "¯",
      Sub: "⋐",
      sub: "⊂",
      subdot: "⪽",
      subE: "⫅",
      sube: "⊆",
      subedot: "⫃",
      submult: "⫁",
      subnE: "⫋",
      subne: "⊊",
      subplus: "⪿",
      subrarr: "⥹",
      Subset: "⋐",
      subset: "⊂",
      subseteq: "⊆",
      subseteqq: "⫅",
      SubsetEqual: "⊆",
      subsetneq: "⊊",
      subsetneqq: "⫋",
      subsim: "⫇",
      subsub: "⫕",
      subsup: "⫓",
      succ: "≻",
      succapprox: "⪸",
      succcurlyeq: "≽",
      Succeeds: "≻",
      SucceedsEqual: "⪰",
      SucceedsSlantEqual: "≽",
      SucceedsTilde: "≿",
      succeq: "⪰",
      succnapprox: "⪺",
      succneqq: "⪶",
      succnsim: "⋩",
      succsim: "≿",
      SuchThat: "∋",
      Sum: "∑",
      sum: "∑",
      sung: "♪",
      Sup: "⋑",
      sup: "⊃",
      sup1: "¹",
      sup2: "²",
      sup3: "³",
      supdot: "⪾",
      supdsub: "⫘",
      supE: "⫆",
      supe: "⊇",
      supedot: "⫄",
      Superset: "⊃",
      SupersetEqual: "⊇",
      suphsol: "⟉",
      suphsub: "⫗",
      suplarr: "⥻",
      supmult: "⫂",
      supnE: "⫌",
      supne: "⊋",
      supplus: "⫀",
      Supset: "⋑",
      supset: "⊃",
      supseteq: "⊇",
      supseteqq: "⫆",
      supsetneq: "⊋",
      supsetneqq: "⫌",
      supsim: "⫈",
      supsub: "⫔",
      supsup: "⫖",
      swarhk: "⤦",
      swArr: "⇙",
      swarr: "↙",
      swarrow: "↙",
      swnwar: "⤪",
      szlig: "ß",
      Tab: "	",
      target: "⌖",
      Tau: "Τ",
      tau: "τ",
      tbrk: "⎴",
      Tcaron: "Ť",
      tcaron: "ť",
      Tcedil: "Ţ",
      tcedil: "ţ",
      Tcy: "Т",
      tcy: "т",
      tdot: "⃛",
      telrec: "⌕",
      Tfr: "𝔗",
      tfr: "𝔱",
      there4: "∴",
      Therefore: "∴",
      therefore: "∴",
      Theta: "Θ",
      theta: "θ",
      thetasym: "ϑ",
      thetav: "ϑ",
      thickapprox: "≈",
      thicksim: "∼",
      ThickSpace: "  ",
      thinsp: " ",
      ThinSpace: " ",
      thkap: "≈",
      thksim: "∼",
      THORN: "Þ",
      thorn: "þ",
      Tilde: "∼",
      tilde: "˜",
      TildeEqual: "≃",
      TildeFullEqual: "≅",
      TildeTilde: "≈",
      times: "×",
      timesb: "⊠",
      timesbar: "⨱",
      timesd: "⨰",
      tint: "∭",
      toea: "⤨",
      top: "⊤",
      topbot: "⌶",
      topcir: "⫱",
      Topf: "𝕋",
      topf: "𝕥",
      topfork: "⫚",
      tosa: "⤩",
      tprime: "‴",
      TRADE: "™",
      trade: "™",
      triangle: "▵",
      triangledown: "▿",
      triangleleft: "◃",
      trianglelefteq: "⊴",
      triangleq: "≜",
      triangleright: "▹",
      trianglerighteq: "⊵",
      tridot: "◬",
      trie: "≜",
      triminus: "⨺",
      TripleDot: "⃛",
      triplus: "⨹",
      trisb: "⧍",
      tritime: "⨻",
      trpezium: "⏢",
      Tscr: "𝒯",
      tscr: "𝓉",
      TScy: "Ц",
      tscy: "ц",
      TSHcy: "Ћ",
      tshcy: "ћ",
      Tstrok: "Ŧ",
      tstrok: "ŧ",
      twixt: "≬",
      twoheadleftarrow: "↞",
      twoheadrightarrow: "↠",
      Uacute: "Ú",
      uacute: "ú",
      Uarr: "↟",
      uArr: "⇑",
      uarr: "↑",
      Uarrocir: "⥉",
      Ubrcy: "Ў",
      ubrcy: "ў",
      Ubreve: "Ŭ",
      ubreve: "ŭ",
      Ucirc: "Û",
      ucirc: "û",
      Ucy: "У",
      ucy: "у",
      udarr: "⇅",
      Udblac: "Ű",
      udblac: "ű",
      udhar: "⥮",
      ufisht: "⥾",
      Ufr: "𝔘",
      ufr: "𝔲",
      Ugrave: "Ù",
      ugrave: "ù",
      uHar: "⥣",
      uharl: "↿",
      uharr: "↾",
      uhblk: "▀",
      ulcorn: "⌜",
      ulcorner: "⌜",
      ulcrop: "⌏",
      ultri: "◸",
      Umacr: "Ū",
      umacr: "ū",
      uml: "¨",
      UnderBar: "_",
      UnderBrace: "⏟",
      UnderBracket: "⎵",
      UnderParenthesis: "⏝",
      Union: "⋃",
      UnionPlus: "⊎",
      Uogon: "Ų",
      uogon: "ų",
      Uopf: "𝕌",
      uopf: "𝕦",
      UpArrow: "↑",
      Uparrow: "⇑",
      uparrow: "↑",
      UpArrowBar: "⤒",
      UpArrowDownArrow: "⇅",
      UpDownArrow: "↕",
      Updownarrow: "⇕",
      updownarrow: "↕",
      UpEquilibrium: "⥮",
      upharpoonleft: "↿",
      upharpoonright: "↾",
      uplus: "⊎",
      UpperLeftArrow: "↖",
      UpperRightArrow: "↗",
      Upsi: "ϒ",
      upsi: "υ",
      upsih: "ϒ",
      Upsilon: "Υ",
      upsilon: "υ",
      UpTee: "⊥",
      UpTeeArrow: "↥",
      upuparrows: "⇈",
      urcorn: "⌝",
      urcorner: "⌝",
      urcrop: "⌎",
      Uring: "Ů",
      uring: "ů",
      urtri: "◹",
      Uscr: "𝒰",
      uscr: "𝓊",
      utdot: "⋰",
      Utilde: "Ũ",
      utilde: "ũ",
      utri: "▵",
      utrif: "▴",
      uuarr: "⇈",
      Uuml: "Ü",
      uuml: "ü",
      uwangle: "⦧",
      vangrt: "⦜",
      varepsilon: "ϵ",
      varkappa: "ϰ",
      varnothing: "∅",
      varphi: "ϕ",
      varpi: "ϖ",
      varpropto: "∝",
      vArr: "⇕",
      varr: "↕",
      varrho: "ϱ",
      varsigma: "ς",
      varsubsetneq: "⊊︀",
      varsubsetneqq: "⫋︀",
      varsupsetneq: "⊋︀",
      varsupsetneqq: "⫌︀",
      vartheta: "ϑ",
      vartriangleleft: "⊲",
      vartriangleright: "⊳",
      Vbar: "⫫",
      vBar: "⫨",
      vBarv: "⫩",
      Vcy: "В",
      vcy: "в",
      VDash: "⊫",
      Vdash: "⊩",
      vDash: "⊨",
      vdash: "⊢",
      Vdashl: "⫦",
      Vee: "⋁",
      vee: "∨",
      veebar: "⊻",
      veeeq: "≚",
      vellip: "⋮",
      Verbar: "‖",
      verbar: "|",
      Vert: "‖",
      vert: "|",
      VerticalBar: "∣",
      VerticalLine: "|",
      VerticalSeparator: "❘",
      VerticalTilde: "≀",
      VeryThinSpace: " ",
      Vfr: "𝔙",
      vfr: "𝔳",
      vltri: "⊲",
      vnsub: "⊂⃒",
      vnsup: "⊃⃒",
      Vopf: "𝕍",
      vopf: "𝕧",
      vprop: "∝",
      vrtri: "⊳",
      Vscr: "𝒱",
      vscr: "𝓋",
      vsubnE: "⫋︀",
      vsubne: "⊊︀",
      vsupnE: "⫌︀",
      vsupne: "⊋︀",
      Vvdash: "⊪",
      vzigzag: "⦚",
      Wcirc: "Ŵ",
      wcirc: "ŵ",
      wedbar: "⩟",
      Wedge: "⋀",
      wedge: "∧",
      wedgeq: "≙",
      weierp: "℘",
      Wfr: "𝔚",
      wfr: "𝔴",
      Wopf: "𝕎",
      wopf: "𝕨",
      wp: "℘",
      wr: "≀",
      wreath: "≀",
      Wscr: "𝒲",
      wscr: "𝓌",
      xcap: "⋂",
      xcirc: "◯",
      xcup: "⋃",
      xdtri: "▽",
      Xfr: "𝔛",
      xfr: "𝔵",
      xhArr: "⟺",
      xharr: "⟷",
      Xi: "Ξ",
      xi: "ξ",
      xlArr: "⟸",
      xlarr: "⟵",
      xmap: "⟼",
      xnis: "⋻",
      xodot: "⨀",
      Xopf: "𝕏",
      xopf: "𝕩",
      xoplus: "⨁",
      xotime: "⨂",
      xrArr: "⟹",
      xrarr: "⟶",
      Xscr: "𝒳",
      xscr: "𝓍",
      xsqcup: "⨆",
      xuplus: "⨄",
      xutri: "△",
      xvee: "⋁",
      xwedge: "⋀",
      Yacute: "Ý",
      yacute: "ý",
      YAcy: "Я",
      yacy: "я",
      Ycirc: "Ŷ",
      ycirc: "ŷ",
      Ycy: "Ы",
      ycy: "ы",
      yen: "¥",
      Yfr: "𝔜",
      yfr: "𝔶",
      YIcy: "Ї",
      yicy: "ї",
      Yopf: "𝕐",
      yopf: "𝕪",
      Yscr: "𝒴",
      yscr: "𝓎",
      YUcy: "Ю",
      yucy: "ю",
      Yuml: "Ÿ",
      yuml: "ÿ",
      Zacute: "Ź",
      zacute: "ź",
      Zcaron: "Ž",
      zcaron: "ž",
      Zcy: "З",
      zcy: "з",
      Zdot: "Ż",
      zdot: "ż",
      zeetrf: "ℨ",
      ZeroWidthSpace: "​",
      Zeta: "Ζ",
      zeta: "ζ",
      Zfr: "ℨ",
      zfr: "𝔷",
      ZHcy: "Ж",
      zhcy: "ж",
      zigrarr: "⇝",
      Zopf: "ℤ",
      zopf: "𝕫",
      Zscr: "𝒵",
      zscr: "𝓏",
      zwj: "‍",
      zwnj: "‌"
    }), n.entityMap = n.HTML_ENTITIES;
  }(gt)), gt;
}
var ht = {}, Rt;
function gr() {
  if (Rt) return ht;
  Rt = 1;
  var n = st(), t = Yt(), u = mt(), s = n.isHTMLEscapableRawTextElement, c = n.isHTMLMimeType, o = n.isHTMLRawTextElement, h = n.hasOwn, C = n.NAMESPACE, f = u.ParseError, g = u.DOMException, D = 0, p = 1, B = 2, k = 3, Y = 4, X = 5, te = 6, M = 7;
  function q() {
  }
  q.prototype = {
    parse: function(l, A, b) {
      var v = this.domBuilder;
      v.startDocument(), U(A, A = /* @__PURE__ */ Object.create(null)), ue(l, A, b, v, this.errorHandler), v.endDocument();
    }
  };
  var W = /&#?\w+;?/g;
  function ue(l, A, b, v, y) {
    var E = c(v.mimeType);
    l.indexOf(t.UNICODE_REPLACEMENT_CHARACTER) >= 0 && y.warning("Unicode replacement character detected, source encoding issues?");
    function S(F) {
      if (F > 65535) {
        F -= 65536;
        var Q = 55296 + (F >> 10), De = 56320 + (F & 1023);
        return String.fromCharCode(Q, De);
      } else
        return String.fromCharCode(F);
    }
    function V(F) {
      var Q = F[F.length - 1] === ";" ? F : F + ";";
      if (!E && Q !== F)
        return y.error("EntityRef: expecting ;"), F;
      var De = t.Reference.exec(Q);
      if (!De || De[0].length !== Q.length)
        return y.error("entity not matching Reference production: " + F), F;
      var ge = Q.slice(1, -1);
      return h(b, ge) ? b[ge] : ge.charAt(0) === "#" ? S(parseInt(ge.substring(1).replace("x", "0x"))) : (y.error("entity not found:" + F), F);
    }
    function O(F) {
      if (F > ie) {
        var Q = l.substring(ie, F).replace(W, V);
        P && oe(ie), v.characters(Q, 0, F - ie), ie = F;
      }
    }
    var T = 0, w = 0, x = /\r\n?|\n|$/g, P = v.locator;
    function oe(F, Q) {
      for (; F >= w && (Q = x.exec(l)); )
        T = w, w = Q.index + Q[0].length, P.lineNumber++;
      P.columnNumber = F - T + 1;
    }
    for (var Te = [{ currentNSMap: A }], Ee = [], ie = 0; ; ) {
      try {
        var R = l.indexOf("<", ie);
        if (R < 0) {
          if (!E && Ee.length > 0)
            return y.fatalError("unclosed xml tag(s): " + Ee.join(", "));
          if (!l.substring(ie).match(/^\s*$/)) {
            var Fe = v.doc, ye = Fe.createTextNode(l.substring(ie));
            if (Fe.documentElement)
              return y.error("Extra content at the end of the document");
            Fe.appendChild(ye), v.currentElement = ye;
          }
          return;
        }
        if (R > ie) {
          var ce = l.substring(ie, R);
          !E && Ee.length === 0 && (ce = ce.replace(new RegExp(t.S_OPT.source, "g"), ""), ce && y.error("Unexpected content outside root element: '" + ce + "'")), O(R);
        }
        switch (l.charAt(R + 1)) {
          case "/":
            var ne = l.indexOf(">", R + 2), Le = l.substring(R + 2, ne > 0 ? ne : void 0);
            if (!Le)
              return y.fatalError("end tag name missing");
            var _e = ne > 0 && t.reg("^", t.QName_group, t.S_OPT, "$").exec(Le);
            if (!_e)
              return y.fatalError('end tag name contains invalid characters: "' + Le + '"');
            if (!v.currentElement && !v.doc.documentElement)
              return;
            var Ce = Ee[Ee.length - 1] || v.currentElement.tagName || v.doc.documentElement.tagName || "";
            if (Ce !== _e[1]) {
              var Re = _e[1].toLowerCase();
              if (!E || Ce.toLowerCase() !== Re)
                return y.fatalError('Opening and ending tag mismatch: "' + Ce + '" != "' + Le + '"');
            }
            var Ue = Te.pop();
            Ee.pop();
            var qe = Ue.localNSMap;
            if (v.endElement(Ue.uri, Ue.localName, Ce), qe)
              for (var Ae in qe)
                h(qe, Ae) && v.endPrefixMapping(Ae);
            ne++;
            break;
          // end element
          case "?":
            P && oe(R), ne = Z(l, R, v, y);
            break;
          case "!":
            P && oe(R), ne = Ne(l, R, v, y, E);
            break;
          default:
            P && oe(R);
            var j = new we(), Pe = Te[Te.length - 1].currentNSMap, ne = m(l, R, j, Pe, V, y, E), Ge = j.length;
            if (j.closed || (E && n.isHTMLVoidElement(j.tagName) ? j.closed = !0 : Ee.push(j.tagName)), P && Ge) {
              for (var at = de(P, {}), Ve = 0; Ve < Ge; Ve++) {
                var ze = j[Ve];
                oe(ze.offset), ze.locator = de(P, {});
              }
              v.locator = at, _(j, v, Pe) && Te.push(j), v.locator = P;
            } else
              _(j, v, Pe) && Te.push(j);
            E && !j.closed ? ne = I(l, ne, j.tagName, V, v) : ne++;
        }
      } catch (F) {
        if (F instanceof f)
          throw F;
        if (F instanceof g)
          throw new f(F.name + ": " + F.message, v.locator, F);
        y.error("element parse error: " + F), ne = -1;
      }
      ne > ie ? ie = ne : O(Math.max(R, ie) + 1);
    }
  }
  function de(l, A) {
    return A.lineNumber = l.lineNumber, A.columnNumber = l.columnNumber, A;
  }
  function m(l, A, b, v, y, E, S) {
    function V(oe, Te, Ee) {
      if (h(b.attributeNames, oe))
        return E.fatalError("Attribute " + oe + " redefined");
      if (!S && Te.indexOf("<") >= 0)
        return E.fatalError("Unescaped '<' not allowed in attributes values");
      b.addValue(
        oe,
        // @see https://www.w3.org/TR/xml/#AVNormalize
        // since the xmldom sax parser does not "interpret" DTD the following is not implemented:
        // - recursive replacement of (DTD) entity references
        // - trimming and collapsing multiple spaces into a single one for attributes that are not of type CDATA
        Te.replace(/[\t\n\r]/g, " ").replace(W, y),
        Ee
      );
    }
    for (var O, T, w = ++A, x = D; ; ) {
      var P = l.charAt(w);
      switch (P) {
        case "=":
          if (x === p)
            O = l.slice(A, w), x = k;
          else if (x === B)
            x = k;
          else
            throw new Error("attribute equal must after attrName");
          break;
        case "'":
        case '"':
          if (x === k || x === p)
            if (x === p && (E.warning('attribute value must after "="'), O = l.slice(A, w)), A = w + 1, w = l.indexOf(P, A), w > 0)
              T = l.slice(A, w), V(O, T, A - 1), x = X;
            else
              throw new Error("attribute value no end '" + P + "' match");
          else if (x == Y)
            T = l.slice(A, w), V(O, T, A), E.warning('attribute "' + O + '" missed start quot(' + P + ")!!"), A = w + 1, x = X;
          else
            throw new Error('attribute value must after "="');
          break;
        case "/":
          switch (x) {
            case D:
              b.setTagName(l.slice(A, w));
            case X:
            case te:
            case M:
              x = M, b.closed = !0;
            case Y:
            case p:
              break;
            case B:
              b.closed = !0;
              break;
            //case S_EQ:
            default:
              throw new Error("attribute invalid close char('/')");
          }
          break;
        case "":
          return E.error("unexpected end of input"), x == D && b.setTagName(l.slice(A, w)), w;
        case ">":
          switch (x) {
            case D:
              b.setTagName(l.slice(A, w));
            case X:
            case te:
            case M:
              break;
            //normal
            case Y:
            //Compatible state
            case p:
              T = l.slice(A, w), T.slice(-1) === "/" && (b.closed = !0, T = T.slice(0, -1));
            case B:
              x === B && (T = O), x == Y ? (E.warning('attribute "' + T + '" missed quot(")!'), V(O, T, A)) : (S || E.warning('attribute "' + T + '" missed value!! "' + T + '" instead!!'), V(T, T, A));
              break;
            case k:
              if (!S)
                return E.fatalError(`AttValue: ' or " expected`);
          }
          return w;
        /*xml space '\x20' | #x9 | #xD | #xA; */
        case "":
          P = " ";
        default:
          if (P <= " ")
            switch (x) {
              case D:
                b.setTagName(l.slice(A, w)), x = te;
                break;
              case p:
                O = l.slice(A, w), x = B;
                break;
              case Y:
                var T = l.slice(A, w);
                E.warning('attribute "' + T + '" missed quot(")!!'), V(O, T, A);
              case X:
                x = te;
                break;
            }
          else
            switch (x) {
              //case S_TAG:void();break;
              //case S_ATTR:void();break;
              //case S_ATTR_NOQUOT_VALUE:void();break;
              case B:
                S || E.warning('attribute "' + O + '" missed value!! "' + O + '" instead2!!'), V(O, O, A), A = w, x = p;
                break;
              case X:
                E.warning('attribute space is required"' + O + '"!!');
              case te:
                x = p, A = w;
                break;
              case k:
                x = Y, A = w;
                break;
              case M:
                throw new Error("elements closed character '/' and '>' must be connected to");
            }
      }
      w++;
    }
  }
  function _(l, A, b) {
    for (var v = l.tagName, y = null, x = l.length; x--; ) {
      var E = l[x], S = E.qName, V = E.value, P = S.indexOf(":");
      if (P > 0)
        var O = E.prefix = S.slice(0, P), T = S.slice(P + 1), w = O === "xmlns" && T;
      else
        T = S, O = null, w = S === "xmlns" && "";
      E.localName = T, w !== !1 && (y == null && (y = /* @__PURE__ */ Object.create(null), U(b, b = /* @__PURE__ */ Object.create(null))), b[w] = y[w] = V, E.uri = C.XMLNS, A.startPrefixMapping(w, V));
    }
    for (var x = l.length; x--; )
      E = l[x], E.prefix && (E.prefix === "xml" && (E.uri = C.XML), E.prefix !== "xmlns" && (E.uri = b[E.prefix]));
    var P = v.indexOf(":");
    P > 0 ? (O = l.prefix = v.slice(0, P), T = l.localName = v.slice(P + 1)) : (O = null, T = l.localName = v);
    var oe = l.uri = b[O || ""];
    if (A.startElement(oe, T, v, l), l.closed) {
      if (A.endElement(oe, T, v), y)
        for (O in y)
          h(y, O) && A.endPrefixMapping(O);
    } else
      return l.currentNSMap = b, l.localNSMap = y, !0;
  }
  function I(l, A, b, v, y) {
    var E = s(b);
    if (E || o(b)) {
      var S = l.indexOf("</" + b + ">", A), V = l.substring(A + 1, S);
      return E && (V = V.replace(W, v)), y.characters(V, 0, V.length), S;
    }
    return A + 1;
  }
  function U(l, A) {
    for (var b in l)
      h(l, b) && (A[b] = l[b]);
  }
  function $(l, A) {
    var b = A;
    function v(w) {
      return w = w || 0, l.charAt(b + w);
    }
    function y(w) {
      w = w || 1, b += w;
    }
    function E() {
      for (var w = 0; b < l.length; ) {
        var x = v();
        if (x !== " " && x !== `
` && x !== "	" && x !== "\r")
          return w;
        w++, y();
      }
      return -1;
    }
    function S() {
      return l.substring(b);
    }
    function V(w) {
      return l.substring(b, b + w.length) === w;
    }
    function O(w) {
      return l.substring(b, b + w.length).toUpperCase() === w.toUpperCase();
    }
    function T(w) {
      var x = t.reg("^", w), P = x.exec(S());
      return P ? (y(P[0].length), P[0]) : null;
    }
    return {
      char: v,
      getIndex: function() {
        return b;
      },
      getMatch: T,
      getSource: function() {
        return l;
      },
      skip: y,
      skipBlanks: E,
      substringFromIndex: S,
      substringStartsWith: V,
      substringStartsWithCaseInsensitive: O
    };
  }
  function J(l, A) {
    function b(V, O) {
      var T = t.PI.exec(V.substringFromIndex());
      return T ? T[1].toLowerCase() === "xml" ? O.fatalError(
        "xml declaration is only allowed at the start of the document, but found at position " + V.getIndex()
      ) : (V.skip(T[0].length), T[0]) : O.fatalError("processing instruction is not well-formed at position " + V.getIndex());
    }
    var v = l.getSource();
    if (l.char() === "[") {
      l.skip(1);
      for (var y = l.getIndex(); l.getIndex() < v.length; ) {
        if (l.skipBlanks(), l.char() === "]") {
          var E = v.substring(y, l.getIndex());
          return l.skip(1), E;
        }
        var S = null;
        if (l.char() === "<" && l.char(1) === "!")
          switch (l.char(2)) {
            case "E":
              l.char(3) === "L" ? S = l.getMatch(t.elementdecl) : l.char(3) === "N" && (S = l.getMatch(t.EntityDecl));
              break;
            case "A":
              S = l.getMatch(t.AttlistDecl);
              break;
            case "N":
              S = l.getMatch(t.NotationDecl);
              break;
            case "-":
              S = l.getMatch(t.Comment);
              break;
          }
        else if (l.char() === "<" && l.char(1) === "?")
          S = b(l, A);
        else if (l.char() === "%")
          S = l.getMatch(t.PEReference);
        else
          return A.fatalError("Error detected in Markup declaration");
        if (!S)
          return A.fatalError("Error in internal subset at position " + l.getIndex());
      }
      return A.fatalError("doctype internal subset is not well-formed, missing ]");
    }
  }
  function Ne(l, A, b, v, y) {
    var E = $(l, A);
    switch (y ? E.char(2).toUpperCase() : E.char(2)) {
      case "-":
        var S = E.getMatch(t.Comment);
        return S ? (b.comment(S, t.COMMENT_START.length, S.length - t.COMMENT_START.length - t.COMMENT_END.length), E.getIndex()) : v.fatalError("comment is not well-formed at position " + E.getIndex());
      case "[":
        var V = E.getMatch(t.CDSect);
        return V ? !y && !b.currentElement ? v.fatalError("CDATA outside of element") : (b.startCDATA(), b.characters(V, t.CDATA_START.length, V.length - t.CDATA_START.length - t.CDATA_END.length), b.endCDATA(), E.getIndex()) : v.fatalError("Invalid CDATA starting at position " + A);
      case "D": {
        if (b.doc && b.doc.documentElement)
          return v.fatalError("Doctype not allowed inside or after documentElement at position " + E.getIndex());
        if (y ? !E.substringStartsWithCaseInsensitive(t.DOCTYPE_DECL_START) : !E.substringStartsWith(t.DOCTYPE_DECL_START))
          return v.fatalError("Expected " + t.DOCTYPE_DECL_START + " at position " + E.getIndex());
        if (E.skip(t.DOCTYPE_DECL_START.length), E.skipBlanks() < 1)
          return v.fatalError("Expected whitespace after " + t.DOCTYPE_DECL_START + " at position " + E.getIndex());
        var O = {
          name: void 0,
          publicId: void 0,
          systemId: void 0,
          internalSubset: void 0
        };
        if (O.name = E.getMatch(t.Name), !O.name)
          return v.fatalError("doctype name missing or contains unexpected characters at position " + E.getIndex());
        if (y && O.name.toLowerCase() !== "html" && v.warning("Unexpected DOCTYPE in HTML document at position " + E.getIndex()), E.skipBlanks(), E.substringStartsWith(t.PUBLIC) || E.substringStartsWith(t.SYSTEM)) {
          var T = t.ExternalID_match.exec(E.substringFromIndex());
          if (!T)
            return v.fatalError("doctype external id is not well-formed at position " + E.getIndex());
          T.groups.SystemLiteralOnly !== void 0 ? O.systemId = T.groups.SystemLiteralOnly : (O.systemId = T.groups.SystemLiteral, O.publicId = T.groups.PubidLiteral), E.skip(T[0].length);
        } else if (y && E.substringStartsWithCaseInsensitive(t.SYSTEM)) {
          if (E.skip(t.SYSTEM.length), E.skipBlanks() < 1)
            return v.fatalError("Expected whitespace after " + t.SYSTEM + " at position " + E.getIndex());
          if (O.systemId = E.getMatch(t.ABOUT_LEGACY_COMPAT_SystemLiteral), !O.systemId)
            return v.fatalError(
              "Expected " + t.ABOUT_LEGACY_COMPAT + " in single or double quotes after " + t.SYSTEM + " at position " + E.getIndex()
            );
        }
        return y && O.systemId && !t.ABOUT_LEGACY_COMPAT_SystemLiteral.test(O.systemId) && v.warning("Unexpected doctype.systemId in HTML document at position " + E.getIndex()), y || (E.skipBlanks(), O.internalSubset = J(E, v)), E.skipBlanks(), E.char() !== ">" ? v.fatalError("doctype not terminated with > at position " + E.getIndex()) : (E.skip(1), b.startDTD(O.name, O.publicId, O.systemId, O.internalSubset), b.endDTD(), E.getIndex());
      }
      default:
        return v.fatalError('Not well-formed XML starting with "<!" at position ' + A);
    }
  }
  function Z(l, A, b, v) {
    var y = l.substring(A).match(t.PI);
    if (!y)
      return v.fatalError("Invalid processing instruction starting at position " + A);
    if (y[1].toLowerCase() === "xml") {
      if (A > 0)
        return v.fatalError(
          "processing instruction at position " + A + " is an xml declaration which is only at the start of the document"
        );
      if (!t.XMLDecl.test(l.substring(A)))
        return v.fatalError("xml declaration is not well-formed");
    }
    return b.processingInstruction(y[1], y[2]), A + y[0].length;
  }
  function we() {
    this.attributeNames = /* @__PURE__ */ Object.create(null);
  }
  return we.prototype = {
    setTagName: function(l) {
      if (!t.QName_exact.test(l))
        throw new Error("invalid tagName:" + l);
      this.tagName = l;
    },
    addValue: function(l, A, b) {
      if (!t.QName_exact.test(l))
        throw new Error("invalid attribute:" + l);
      this.attributeNames[l] = this.length, this[this.length++] = { qName: l, value: A, offset: b };
    },
    length: 0,
    getLocalName: function(l) {
      return this[l].localName;
    },
    getLocator: function(l) {
      return this[l].locator;
    },
    getQName: function(l) {
      return this[l].qName;
    },
    getURI: function(l) {
      return this[l].uri;
    },
    getValue: function(l) {
      return this[l].value;
    }
    //	,getIndex:function(uri, localName)){
    //		if(localName){
    //
    //		}else{
    //			var qName = uri
    //		}
    //	},
    //	getValue:function(){return this.getValue(this.getIndex.apply(this,arguments))},
    //	getType:function(uri,localName){}
    //	getType:function(i){},
  }, ht.XMLReader = q, ht.parseUtils = $, ht.parseDoctypeCommentOrCData = Ne, ht;
}
var Mt;
function Ar() {
  if (Mt) return We;
  Mt = 1;
  var n = st(), t = Xt(), u = mt(), s = Dr(), c = gr(), o = t.DOMImplementation, h = n.hasDefaultHTMLNamespace, C = n.isHTMLMimeType, f = n.isValidMimeType, g = n.MIME_TYPE, D = n.NAMESPACE, p = u.ParseError, B = c.XMLReader;
  function k(m) {
    return m.replace(/\r[\n\u0085]/g, `
`).replace(/[\r\u0085\u2028\u2029]/g, `
`);
  }
  function Y(m) {
    if (m = m || {}, m.locator === void 0 && (m.locator = !0), this.assign = m.assign || n.assign, this.domHandler = m.domHandler || X, this.onError = m.onError || m.errorHandler, m.errorHandler && typeof m.errorHandler != "function")
      throw new TypeError("errorHandler object is no longer supported, switch to onError!");
    m.errorHandler && m.errorHandler("warning", "The `errorHandler` option has been deprecated, use `onError` instead!", this), this.normalizeLineEndings = m.normalizeLineEndings || k, this.locator = !!m.locator, this.xmlns = this.assign(/* @__PURE__ */ Object.create(null), m.xmlns);
  }
  Y.prototype.parseFromString = function(m, _) {
    if (!f(_))
      throw new TypeError('DOMParser.parseFromString: the provided mimeType "' + _ + '" is not valid.');
    var I = this.assign(/* @__PURE__ */ Object.create(null), this.xmlns), U = s.XML_ENTITIES, $ = I[""] || null;
    h(_) ? (U = s.HTML_ENTITIES, $ = D.HTML) : _ === g.XML_SVG_IMAGE && ($ = D.SVG), I[""] = $, I.xml = I.xml || D.XML;
    var J = new this.domHandler({
      mimeType: _,
      defaultNamespace: $,
      onError: this.onError
    }), Ne = this.locator ? {} : void 0;
    this.locator && J.setDocumentLocator(Ne);
    var Z = new B();
    Z.errorHandler = J, Z.domBuilder = J;
    var we = !n.isHTMLMimeType(_);
    return we && typeof m != "string" && Z.errorHandler.fatalError("source is not a string"), Z.parse(this.normalizeLineEndings(String(m)), I, U), J.doc.documentElement || Z.errorHandler.fatalError("missing root element"), J.doc;
  };
  function X(m) {
    var _ = m || {};
    this.mimeType = _.mimeType || g.XML_APPLICATION, this.defaultNamespace = _.defaultNamespace || null, this.cdata = !1, this.currentElement = void 0, this.doc = void 0, this.locator = void 0, this.onError = _.onError;
  }
  function te(m, _) {
    _.lineNumber = m.lineNumber, _.columnNumber = m.columnNumber;
  }
  X.prototype = {
    /**
     * Either creates an XML or an HTML document and stores it under `this.doc`.
     * If it is an XML document, `this.defaultNamespace` is used to create it,
     * and it will not contain any `childNodes`.
     * If it is an HTML document, it will be created without any `childNodes`.
     *
     * @see http://www.saxproject.org/apidoc/org/xml/sax/ContentHandler.html
     */
    startDocument: function() {
      var m = new o();
      this.doc = C(this.mimeType) ? m.createHTMLDocument(!1) : m.createDocument(this.defaultNamespace, "");
    },
    startElement: function(m, _, I, U) {
      var $ = this.doc, J = $.createElementNS(m, I || _), Ne = U.length;
      W(this, J), this.currentElement = J, this.locator && te(this.locator, J);
      for (var Z = 0; Z < Ne; Z++) {
        var m = U.getURI(Z), we = U.getValue(Z), I = U.getQName(Z), l = $.createAttributeNS(m, I);
        this.locator && te(U.getLocator(Z), l), l.value = l.nodeValue = we, J.setAttributeNode(l);
      }
    },
    endElement: function(m, _, I) {
      this.currentElement = this.currentElement.parentNode;
    },
    startPrefixMapping: function(m, _) {
    },
    endPrefixMapping: function(m) {
    },
    processingInstruction: function(m, _) {
      var I = this.doc.createProcessingInstruction(m, _);
      this.locator && te(this.locator, I), W(this, I);
    },
    ignorableWhitespace: function(m, _, I) {
    },
    characters: function(m, _, I) {
      if (m = q.apply(this, arguments), m) {
        if (this.cdata)
          var U = this.doc.createCDATASection(m);
        else
          var U = this.doc.createTextNode(m);
        this.currentElement ? this.currentElement.appendChild(U) : /^\s*$/.test(m) && this.doc.appendChild(U), this.locator && te(this.locator, U);
      }
    },
    skippedEntity: function(m) {
    },
    endDocument: function() {
      this.doc.normalize();
    },
    /**
     * Stores the locator to be able to set the `columnNumber` and `lineNumber`
     * on the created DOM nodes.
     *
     * @param {Locator} locator
     */
    setDocumentLocator: function(m) {
      m && (m.lineNumber = 0), this.locator = m;
    },
    //LexicalHandler
    comment: function(m, _, I) {
      m = q.apply(this, arguments);
      var U = this.doc.createComment(m);
      this.locator && te(this.locator, U), W(this, U);
    },
    startCDATA: function() {
      this.cdata = !0;
    },
    endCDATA: function() {
      this.cdata = !1;
    },
    startDTD: function(m, _, I, U) {
      var $ = this.doc.implementation;
      if ($ && $.createDocumentType) {
        var J = $.createDocumentType(m, _, I, U);
        this.locator && te(this.locator, J), W(this, J), this.doc.doctype = J;
      }
    },
    reportError: function(m, _) {
      if (typeof this.onError == "function")
        try {
          this.onError(m, _, this);
        } catch (I) {
          throw new p("Reporting " + m + ' "' + _ + '" caused ' + I, this.locator);
        }
      else
        console.error("[xmldom " + m + "]	" + _, M(this.locator));
    },
    /**
     * @see http://www.saxproject.org/apidoc/org/xml/sax/ErrorHandler.html
     */
    warning: function(m) {
      this.reportError("warning", m);
    },
    error: function(m) {
      this.reportError("error", m);
    },
    /**
     * This function reports a fatal error and throws a ParseError.
     *
     * @param {string} message
     * - The message to be used for reporting and throwing the error.
     * @returns {never}
     * This function always throws an error and never returns a value.
     * @throws {ParseError}
     * Always throws a ParseError with the provided message.
     */
    fatalError: function(m) {
      throw this.reportError("fatalError", m), new p(m, this.locator);
    }
  };
  function M(m) {
    if (m)
      return `
@#[line:` + m.lineNumber + ",col:" + m.columnNumber + "]";
  }
  function q(m, _, I) {
    return typeof m == "string" ? m.substr(_, I) : m.length >= _ + I || _ ? new java.lang.String(m, _, I) + "" : m;
  }
  "endDTD,startEntity,endEntity,attributeDecl,elementDecl,externalEntityDecl,internalEntityDecl,resolveEntity,getExternalSubset,notationDecl,unparsedEntityDecl".replace(
    /\w+/g,
    function(m) {
      X.prototype[m] = function() {
        return null;
      };
    }
  );
  function W(m, _) {
    m.currentElement ? m.currentElement.appendChild(_) : m.doc.appendChild(_);
  }
  function ue(m) {
    if (m === "error") throw "onErrorStopParsing";
  }
  function de() {
    throw "onWarningStopParsing";
  }
  return We.__DOMHandler = X, We.DOMParser = Y, We.normalizeLineEndings = k, We.onErrorStopParsing = ue, We.onWarningStopParsing = de, We;
}
var xt;
function vr() {
  if (xt) return G;
  xt = 1;
  var n = st();
  G.assign = n.assign, G.hasDefaultHTMLNamespace = n.hasDefaultHTMLNamespace, G.isHTMLMimeType = n.isHTMLMimeType, G.isValidMimeType = n.isValidMimeType, G.MIME_TYPE = n.MIME_TYPE, G.NAMESPACE = n.NAMESPACE;
  var t = mt();
  G.DOMException = t.DOMException, G.DOMExceptionName = t.DOMExceptionName, G.ExceptionCode = t.ExceptionCode, G.ParseError = t.ParseError;
  var u = Xt();
  G.Attr = u.Attr, G.CDATASection = u.CDATASection, G.CharacterData = u.CharacterData, G.Comment = u.Comment, G.Document = u.Document, G.DocumentFragment = u.DocumentFragment, G.DocumentType = u.DocumentType, G.DOMImplementation = u.DOMImplementation, G.Element = u.Element, G.Entity = u.Entity, G.EntityReference = u.EntityReference, G.LiveNodeList = u.LiveNodeList, G.NamedNodeMap = u.NamedNodeMap, G.Node = u.Node, G.NodeList = u.NodeList, G.Notation = u.Notation, G.ProcessingInstruction = u.ProcessingInstruction, G.Text = u.Text, G.XMLSerializer = u.XMLSerializer;
  var s = Ar();
  return G.DOMParser = s.DOMParser, G.normalizeLineEndings = s.normalizeLineEndings, G.onErrorStopParsing = s.onErrorStopParsing, G.onWarningStopParsing = s.onWarningStopParsing, G;
}
var It = vr();
class Tr {
  constructor({ xml: t }) {
    this.parent = null, this.child = null, this.minval = NaN, this.maxval = NaN, this.origin = new pt(), this.axis = new Oe({
      x: 1,
      y: 0,
      z: 0
    }), this.name = t.getAttribute(H.Name) ?? "unknown_name", this.type = t.getAttribute(H.Type);
    const u = t.getElementsByTagName(H.Parent);
    u[0] && (this.parent = u[0].getAttribute(H.Link));
    const s = t.getElementsByTagName(H.Child);
    s[0] && (this.child = s[0].getAttribute(H.Link));
    const c = t.getElementsByTagName(H.Limit);
    c[0] && (this.minval = parseFloat(
      c[0].getAttribute(H.Lower) ?? "NaN"
    ), this.maxval = parseFloat(
      c[0].getAttribute(H.Upper) ?? "NaN"
    ));
    const o = t.getElementsByTagName(H.Origin);
    o[0] && (this.origin = zt(o[0]));
    const h = t.getElementsByTagName(H.Axis);
    if (h[0]) {
      const C = h[0].getAttribute(H.Xyz)?.split(" ");
      if (C?.length !== 3)
        throw new Error(
          "If specified, axis must have an xyz value composed of three numbers"
        );
      const [f, g, D] = C.map(parseFloat);
      this.axis = new Oe({
        x: f,
        y: g,
        z: D
      });
    }
  }
}
class qr {
  constructor({ xml: t, string: u }) {
    this.materials = {}, this.links = {}, this.joints = {};
    let s = t;
    if (u && (s = new It.DOMParser().parseFromString(u, It.MIME_TYPE.XML_TEXT).documentElement ?? void 0), !s)
      throw new Error("No URDF document parsed!");
    this.name = s.getAttribute(H.Name);
    const c = s.childNodes;
    for (const o of c)
      if (Ht(o))
        switch (o.tagName) {
          case "material": {
            const h = new Vt({ xml: o });
            if (!Object.hasOwn(this.materials, h.name)) {
              this.materials[h.name] = h;
              break;
            }
            const C = this.materials[h.name];
            C?.isLink() ? C.assign(h) : console.warn(`Material ${h.name} is not unique.`);
            break;
          }
          case "link": {
            const h = new Er({ xml: o });
            if (Object.hasOwn(this.links, h.name)) {
              console.warn(`Link ${h.name} is not unique.`);
              break;
            }
            for (const C of h.visuals) {
              const f = C.material;
              if (!f?.name)
                continue;
              const g = this.materials[f.name];
              g ? C.material = g : this.materials[f.name] = f;
            }
            this.links[h.name] = h;
            break;
          }
          case "joint": {
            const h = new Tr({ xml: o });
            this.joints[h.name] = h;
            break;
          }
        }
  }
}
const Gr = "2.0.1";
export {
  Pr as AbstractTransport,
  nr as Action,
  qt as ActionClient,
  kr as ActionListener,
  er as Goal,
  Be as GoalStatus,
  Kt as Param,
  pt as Pose,
  Qe as Quaternion,
  Gr as REVISION,
  Ur as ROS2TFClient,
  Lr as Ros,
  re as Service,
  rr as SimpleActionServer,
  tr as TFClient,
  fe as Topic,
  vt as Transform,
  H as UrdfAttrs,
  cr as UrdfBox,
  lr as UrdfColor,
  hr as UrdfCylinder,
  Tr as UrdfJoint,
  Er as UrdfLink,
  Vt as UrdfMaterial,
  pr as UrdfMesh,
  qr as UrdfModel,
  fr as UrdfSphere,
  ft as UrdfType,
  mr as UrdfVisual,
  Oe as Vector3,
  ur as WebSocketTransportFactory,
  Ht as isElement,
  kt as isRosbridgeActionFeedbackMessage,
  Ut as isRosbridgeActionResultMessage,
  Br as isRosbridgeAdvertiseActionMessage,
  Sr as isRosbridgeAdvertiseMessage,
  xr as isRosbridgeAdvertiseServiceMessage,
  At as isRosbridgeCallServiceMessage,
  Pt as isRosbridgeCancelActionGoalMessage,
  Jt as isRosbridgeFragmentMessage,
  ut as isRosbridgeMessage,
  Zt as isRosbridgePngMessage,
  Bt as isRosbridgePublishMessage,
  Lt as isRosbridgeSendActionGoalMessage,
  Ft as isRosbridgeServiceResponseMessage,
  _r as isRosbridgeSetStatusLevelMessage,
  Qt as isRosbridgeStatusMessage,
  Rr as isRosbridgeSubscribeMessage,
  Fr as isRosbridgeUnadvertiseActionMessage,
  Or as isRosbridgeUnadvertiseMessage,
  Ir as isRosbridgeUnadvertiseServiceMessage,
  Mr as isRosbridgeUnsubscribeMessage,
  zt as parseUrdfOrigin
};
