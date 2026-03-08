import { writable, type Writable } from "svelte/store";
import * as ROSLIB from "roslib";
import { toast } from "svelte-sonner";

import type * as RJ from "$lib/rosTypes";
import { play_state_to_ros, play_state_from_ros, match_state_from_ros, to_kicker_status, KickerStatus, State, Restart, Period, ShootMode, TriggerMode, control_command_from_msg } from "$lib/rosTypes";

let ros: ROSLIB.Ros;
let connectionToastId: string | number | null = null;
let topics = new Map<string, ROSLIB.Topic<any>>();

export const fieldDimensions: Writable<RJ.FieldDimensions> = writable({
    length: 9.0,
    width: 6.0,
    border: 0.7,
    line_width: 0.01,
    goal_width: 1.0,
    goal_depth: 0.18,
    goal_height: 0.16,
    penalty_short_dist: 1.0,
    penalty_long_dist: 2.0,
    center_radius: 0.5,
    center_diameter: 1.0,
    goal_float: 0.5,
    floor_length: 10.4,
    floor_width: 7.4
});

export const robotStatuses: Writable<Array<RJ.RobotStatus>> = writable(Array.from({ length: 16 }, (_, i) => ({
    robot_id: i,
    alive: false,
    kicker_status: KickerStatus.Unhealthy,
    has_ball_sense: false,
    battery_percent: 0.0,
    position: "unknown"
})));

export const worldState: Writable<RJ.WorldStateMsg> = writable({
    last_update_time: { sec: 0, nanosec: 0 },
    their_robots: Array.from({ length: 16 }, (_, i) => ({
        stamp: { sec: 0, nanosec: 0 },
        pose: {
            position: { x: 0, y: 0 },
            heading: 0
        },
        velocity: {
            linear: { x: 0, y: 0 },
            angular: 0
        },
        visible: false
    })),
    our_robots: Array.from({ length: 16 }, (_, i) => ({
        stamp: { sec: 0, nanosec: 0 },
        pose: {
            position: { x: 0, y: 0 },
            heading: 0
        },
        velocity: {
            linear: { x: 0, y: 0 },
            angular: 0
        },
        visible: false
    })),
    ball: {
        stamp: { sec: 0, nanosec: 0 },
        position: { x: 0, y: 0 },
        velocity: { x: 0, y: 0},
        visible: false
    }
});

export const playState: Writable<RJ.PlayState> = writable({
    state: State.Halt,
    restart: Restart.Kickoff,
    our_restart: true,
    placement_point: { x: 0, y: 0 }
});

export const matchState: Writable<RJ.MatchState> = writable({
    period: Period.FirstHalf,
    stage_time_left: { sec: 0, nanosec: 0 }
});

export const goalie: Writable<RJ.GoalieMsg> = writable({
    goalie_id: 0,
});

export const teamColor: Writable<RJ.TeamColorMsg> = writable({
    is_blue: true,
});

export const ourTeamInfo: Writable<RJ.TeamInfo> = writable({
    name: "Unkonwn",
    score: 0,
    num_red_cards: 0,
    num_yellow_cards: 0,
    yellow_card_remaining_times: [],
    timeouts_left: 0,
    remaining_timeout_time: { sec: 0, nanosec: 0 },
    goalie_id: 0
});

export const theirTeamInfo: Writable<RJ.TeamInfo> = writable({
    name: "Unknown",
    score: 0,
    num_red_cards: 0,
    num_yellow_cards: 0,
    yellow_card_remaining_times: [],
    timeouts_left: 0,
    remaining_timeout_time: { sec: 0, nanosec: 0 },
    goalie_id: 0
});

export const gameSettings: Writable<RJ.GameSettings> = writable({
    simulation: true,
    request_blue_team: false,
    request_goalie_id: 0,
    defend_plus_x: true,
    use_our_half: true,
    use_their_half: true,
    paused: false
});

export const controlCommands: Writable<Array<RJ.ControlCommand>> = writable(Array.from({ length: 16 }, (_, i) => ({
    shoot_mode: ShootMode.Kick,
    trigger_mode: TriggerMode.StandDown,
    kick_strength: 0,
    dribble_speed: 0,
    velocity: { linear: { x: 0, y: 0}, angular: 0}
})));

export function initROS(url: string = "ws://localhost:9090") {
    ros = new ROSLIB.Ros({ url });

    ros.on("connection", () => {
        if (connectionToastId) {
            toast.success("ROS Bridge Connected", {
                id: connectionToastId,
                description: "Communication with RJ Soccer Established",
                duration: 3000
            });
            connectionToastId = null;
        }

        subscribeToRosTopics();
    });

    ros.on("error", () => {
        if (!connectionToastId) {
            connectionToastId = toast.error("Connection Error", {
                description: "Could not reach ROS Bridge",
                duration: Infinity
            });
        }

        unsubscribeFromRosTopics();
    });

    ros.on("close", () => {
        if (!connectionToastId) {
            connectionToastId = toast.warning("Connection Lost", {
                description: "WebSocket closed. Attempting to reconnect...",
                duration: Infinity
            });
        }

        unsubscribeFromRosTopics();

        setTimeout(() => initROS(url), 2000);
    });
}

function subscribeToRosTopics() {
    topics.set("world_state", new ROSLIB.Topic({
        ros: ros,
        name: "/vision_filter/world_state",
        messageType: "rj_msgs/msg/WorldState"
    }));
    topics.get("world_state")?.subscribe((msg: RJ.WorldStateMsg) => {
        worldState.set(msg);
    });

    topics.set("play_state", new ROSLIB.Topic({
        ros: ros,
        name: "/referee/play_state",
        messageType: "rj_msgs/msg/PlayState"
    }));
    topics.get("play_state")?.subscribe((msg: RJ.PlayStateMsg) => {
        playState.set(play_state_from_ros(msg));
    });

    topics.set("match_state", new ROSLIB.Topic({
        ros: ros,
        name: "/referee/match_state",
        messageType: "rj_msgs/msg/MatchState"
    }));
    topics.get("match_state")?.subscribe((msg: RJ.MatchStateMsg) => {
        matchState.set(match_state_from_ros(msg));
    });

    topics.set("team_color", new ROSLIB.Topic({
        ros: ros,
        name: "/referee/team_color",
        messageType: "rj_msgs/msg/TeamColor"
    }));
    topics.get("team_color")?.subscribe((msg: RJ.TeamColorMsg) => {
        teamColor.set(msg);
    });

    topics.set("goalie", new ROSLIB.Topic({
        ros: ros,
        name: "/referee/our_goalie",
        messageType: "rj_msgs/msg/Goalie"
    }));
    topics.get("goalie")?.subscribe((msg: RJ.GoalieMsg) => {
        goalie.set(msg);
    });

    topics.set("our_team_info", new ROSLIB.Topic({
        ros: ros,
        name: "/referee/our_info",
        messageType: "rj_msgs/msg/TeamInfo"
    }));
    topics.get("our_team_info")?.subscribe((msg: RJ.TeamInfo) => {
        ourTeamInfo.set(msg);
    });

    topics.set("their_team_info", new ROSLIB.Topic({
        ros: ros,
        name: "/referee/their_info",
        messageType: "rj_msgs/msg/TeamInfo"
    }));
    topics.get("their_team_info")?.subscribe((msg: RJ.TeamInfo) => {
        theirTeamInfo.set(msg);
    });

    topics.set("game_settings", new ROSLIB.Topic({
        ros: ros,
        name: "/config/game_settings",
        messageType: "rj_msgs/msg/GameSettings"
    }));
    topics.get("game_settings")?.subscribe((msg: RJ.GameSettings) => {
        gameSettings.set(msg);
    });

    for (let robotId = 0; robotId < 16; robotId++) {
        topics.set(`robot_${robotId}_status`, new ROSLIB.Topic({
            ros: ros,
            name: `/radio/robot_status/robot_${robotId}`,
            messageType: "rj_msgs/msg/RobotStatus"
        }));
        topics.get(`robot_${robotId}_status`)?.subscribe((msg: RJ.RobotStatusMsg) => {
            robotStatuses.update(state => {
                state[msg.robot_id].kicker_status = to_kicker_status(msg.kicker_healthy, msg.kicker_charged);
                state[msg.robot_id].has_ball_sense = msg.has_ball_sense;
                state[msg.robot_id].battery_percent = msg.battery_percent;
                return state;
            });
        });

        topics.set(`robot_${robotId}_control_command`, new ROSLIB.Topic({
            ros: ros,
            name: `/robot_${robotId}/control`,
            messageType: "rj_control_msgs/msg/ControlCommand"
        }));
        topics.get(`robot_${robotId}_control_command`)?.subscribe((msg: RJ.ControlCommandMsg) => {
            controlCommands.update(state => {
                state[robotId] = control_command_from_msg(msg);
                return state;
            });
        });
    }

    topics.set("field_dimensions", new ROSLIB.Topic({
        ros: ros,
        name: "/config/field_dimensions",
        messageType: "rj_msgs/msg/FieldDimensions"
    }));
    topics.get("field_dimensions")?.subscribe((msg: RJ.FieldDimensions) => {
        fieldDimensions.set(msg);
    });

    topics.set("alive_robots", new ROSLIB.Topic({
        ros: ros,
        name: "/radio/alive_robots",
        messageType: "rj_msgs/msg/AliveRobots"
    }));
    topics.get("alive_robots")?.subscribe((msg: RJ.AliveRobotsMsg) => {
        robotStatuses.update(state => {
            for (let i = 0; i < 16; i++) {
                state[i].alive = msg.alive_robots[i];
            }
            return state;
        });
    });
}

function unsubscribeFromRosTopics() {
    topics.forEach((topic, _) => {
        topic.unsubscribe();
    });
    topics.clear();
}

export function teleportBall(position: RJ.Point) {
    const topic = new ROSLIB.Topic({
        ros: ros,
        name: "/sim/placement",
        messageType: "/rj_msgs/msg/SimPlacement"
    });

    const placement_msg: RJ.SimPlacement = {
        robots: [],
        ball: {
            position: [position],
            velocity: [{ x: 0, y: 0}]
        }
    };
    topic.publish(placement_msg);
}

export function teleportRobot(blueTeam: boolean, robotId: number, pose: RJ.Pose) {
    const topic = new ROSLIB.Topic({
        ros: ros,
        name: "/sim/placement",
        messageType: "/rj_msgs/msg/SimPlacement"
    });

    const placement_msg: RJ.SimPlacement = {
        robots: [{
            is_blue_team: blueTeam,
            robot_id: robotId,
            pose: pose,
            velocity: { linear: { x: 0, y: 0 }, angular: 0 }
        }],
        ball: {
            position: [],
            velocity: []
        }
    };
    topic.publish(placement_msg);
}

export function setGameSettings(game_settings: RJ.GameSettings) {
    const client = new ROSLIB.Service({
        ros: ros,
        name: "/config/set_game_settings",
        serviceType: "/rj_msgs/srv/SetGameSettings"
    });
    client.callService({ game_settings: game_settings }, (_: any) => {});
}

export function setPlayState(play_state: RJ.PlayState) {
    const client = new ROSLIB.Service({
        ros: ros,
        name: "/referee/quick_commands",
        serviceType: "rj_msgs/srv/QuickCommands"
    });
    client.callService({ command: play_state_to_ros(play_state) }, (_: any) => {});
}