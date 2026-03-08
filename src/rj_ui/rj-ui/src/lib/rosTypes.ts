/**
 * Common types for the robojackets ui
 */

export type Time = {
    sec: number;
    nanosec: number;
};

export type Duration = {
    sec: number;
    nanosec: number;
};

export type Point = {
    x: number;
    y: number;
};

export type Pose = {
    position: Point,
    heading: number;
};

export type Twist = {
    linear: Point;
    angular: number;
};

export enum State {
    Halt,
    Stop,
    Setup,
    Ready,
    Playing,
    PenaltyPlaying
};
const numToState: Record<number, State> = {
    0: State.Halt,
    1: State.Stop,
    2: State.Setup,
    3: State.Ready,
    4: State.Playing,
    5: State.PenaltyPlaying
};

export enum Restart {
    None,
    Kickoff,
    Free,
    Penalty,
    Placement
};
const numToRestart: Record<number, Restart> = {
    0: Restart.None,
    1: Restart.Kickoff,
    2: Restart.Free,
    3: Restart.Penalty,
    4: Restart.Placement
};

export type PlayStateMsg = {
    state: number;
    restart: number;
    our_restart: boolean;
    placement_point: Point
};

export type PlayState = {
    state: State;
    restart: Restart;
    our_restart: boolean;
    placement_point: Point
};

export function play_state_from_ros(msg: PlayStateMsg): PlayState {
    return {
        state: numToState[msg.state],
        restart: numToRestart[msg.restart],
        our_restart: msg.our_restart,
        placement_point: msg.placement_point
    };
}

export function play_state_to_ros(play_state: PlayState): PlayStateMsg {
    return {
        state: play_state.state.valueOf(),
        restart: play_state.restart.valueOf(),
        our_restart: play_state.our_restart,
        placement_point: play_state.placement_point   
    };
}

export enum Period {
    FirstHalf,
    Halftime,
    SecondHalf,
    Overtime1,
    Overtime2,
    PenaltyShootout
};
const numToPeriod: Record<number, Period> = {
    0: Period.FirstHalf,
    1: Period.Halftime,
    2: Period.SecondHalf,
    3: Period.Overtime1,
    4: Period.Overtime2,
    5: Period.PenaltyShootout
};

export function periodToString(period: Period): string {
    switch (period) {
        case Period.FirstHalf:
            return "First Half";
        case Period.Halftime:
            return "Halftime";
        case Period.SecondHalf:
            return "Second Half";
        case Period.Overtime1:
            return "Overtime 1";
        case Period.Overtime2:
            return "Overtime 2";
        case Period.PenaltyShootout:
            return "Penalties";
    }
}

export type MatchStateMsg = {
    period: number;
    stage_time_left: Duration;
};

export type MatchState = {
    period: Period;
    stage_time_left: Duration;
};

export function match_state_from_ros(msg: MatchStateMsg): MatchState {
    return {
        period: numToPeriod[msg.period],
        stage_time_left: msg.stage_time_left,
    };
}

export type GoalieMsg = {
    goalie_id: number;
};

export type TeamColorMsg = {
    is_blue: boolean;
}

export type TeamInfo = {
    name: string;
    score: number;
    num_red_cards: number;
    num_yellow_cards: number;
    yellow_card_remaining_times: Array<Duration>;
    timeouts_left: number;
    remaining_timeout_time: Duration
    goalie_id: number;
};

export type RobotPlacement = {
    is_blue_team: boolean;
    robot_id: number;
    pose: Pose;
    velocity: Twist;
};

export type BallPlacement = {
    position: Array<Point>;
    velocity: Array<Point>;
};

export type SimPlacement = {
    robots: Array<RobotPlacement>;
    ball: BallPlacement;
};

export type GameSettings = {
    simulation: boolean;
    request_blue_team: boolean;
    request_goalie_id: number;
    defend_plus_x: boolean;
    use_our_half: boolean;
    use_their_half: boolean;
    paused: boolean;
};

export type RobotStateMsg = {
    stamp: Time;
    pose: Pose;
    velocity: Twist;
    visible: boolean;
};

export type RobotStatusMsg = {
    timestamp: Time;
    robot_id: number;
    kicker_healthy: boolean;
    kicker_charged: boolean;
    has_ball_sense: boolean
    blue_team: boolean;
    battery_percent: number;
    motor_errors: Array<boolean>;
};

export enum KickerStatus {
    Unhealthy,
    Uncharged,
    Charged
};

export function to_kicker_status(kicker_healthy: boolean, kicker_charged: boolean): KickerStatus {
    if (!kicker_healthy) {
        return KickerStatus.Unhealthy;
    }

    if (kicker_charged) {
        return KickerStatus.Charged;
    }

    return KickerStatus.Uncharged;
}

export type RobotStatus = {
    robot_id: number;
    alive: boolean;
    kicker_status: KickerStatus;
    has_ball_sense: boolean;
    battery_percent: number;
    position: string;
};

export type AliveRobotsMsg = {
    alive_robots: Array<boolean>;
};

export type BallState = {
    stamp: Time;
    position: Point;
    velocity: Point;
    visible: boolean;
};

export type WorldStateMsg = {
    last_update_time: Time;
    their_robots: Array<RobotStateMsg>;
    our_robots: Array<RobotStateMsg>;
    ball: BallState;
};

export type FieldDimensions = {
    length: number;
    width: number;
    border: number;
    line_width: number;
    goal_width: number;
    goal_depth: number;
    goal_height: number;
    penalty_short_dist: number;
    penalty_long_dist: number;
    center_radius: number;
    center_diameter: number;
    goal_float: number;
    floor_length: number;
    floor_width: number;
};

export enum ShootMode {
    Kick,
    Chip
};
const numToShootMode: Record<number, ShootMode> = {
    0: ShootMode.Kick,
    1: ShootMode.Chip,
};

export enum TriggerMode {
    StandDown,
    Immediate,
    OnBreakBeam
};
const numToTriggerMode: Record<number, TriggerMode> = {
    0: TriggerMode.StandDown,
    1: TriggerMode.Immediate,
    2: TriggerMode.OnBreakBeam
};

export type ControlCommandMsg = {
    shoot_mode: number;
    trigger_mode: number;
    kick_strength: number;
    dribble_speed: number;
    velocity: Twist;
};

export type ControlCommand = {
    shoot_mode: ShootMode;
    trigger_mode: TriggerMode;
    kick_strength: number;
    dribble_speed: number;
    velocity: Twist;
};

export function control_command_from_msg(msg: ControlCommandMsg): ControlCommand {
    return {
        shoot_mode: numToShootMode[msg.shoot_mode],
        trigger_mode: numToTriggerMode[msg.trigger_mode],
        kick_strength: msg.kick_strength,
        dribble_speed: msg.dribble_speed,
        velocity: msg.velocity
    };
}