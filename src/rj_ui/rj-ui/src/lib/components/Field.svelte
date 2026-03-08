<script lang="ts">
  import type { FieldDimensions, WorldStateMsg, TeamColorMsg, GameSettings } from '$lib/rosTypes';

  export let fieldDimensions: FieldDimensions;
  export let worldState: WorldStateMsg;
  export let teamColor: TeamColorMsg;
  export let gameSettings: GameSettings;

  export let teleport = (type: 'our-robot' | 'their-robot' | 'ball', id: number | null = null, x: number, y: number) => {};

  let svgElement: SVGSVGElement;

  // Interaction State
  let isDragging = false;
  let dragTarget: { type: 'our-robot' | 'their-robot' | 'ball'; id?: number } | null = null;
  let ghostPos = { x: 0, y: 0 };

  const ROBOT_RADIUS = 90;
  const BALL_RADIUS = 45;
  const CLICK_THRESHOLD = 150; // Distance in mm to "grab" an object

  const mToMM = (m: number) => { return m * 1000; };

  $: viewBox = `-${mToMM(fieldDimensions.floor_length) / 2} -${mToMM(fieldDimensions.floor_width) / 2} ${mToMM(fieldDimensions.floor_length)} ${mToMM(fieldDimensions.floor_width)}`;

  function getSVGCoords(event: MouseEvent) {
    const pt = svgElement.createSVGPoint();
    pt.x = event.clientX;
    pt.y = event.clientY;
    return pt.matrixTransform(svgElement.getScreenCTM()?.inverse());
  }

  function handleMouseDown(e: MouseEvent) {
    const coords = getSVGCoords(e);
    
    // 1. Check if clicking ball
    const distToBall = Math.hypot(coords.x - mToMM(worldState.ball.position.x), coords.y - mToMM(worldState.ball.position.y));
    if (worldState.ball.visible && distToBall < CLICK_THRESHOLD) {
      dragTarget = { type: 'ball' };
      isDragging = true;
      ghostPos = { ...coords };
      return;
    }

    // 2. Check if clicking our robots
    for (let i = 0; i < worldState.our_robots.length; i++) {
      const bot = worldState.our_robots[i];
      if (!bot.visible) continue;
      const dist = Math.hypot(coords.x - mToMM(bot.pose.position.x), coords.y - mToMM(bot.pose.position.y));
      if (dist < CLICK_THRESHOLD) {
        dragTarget = { type: 'our-robot', id: i };
        isDragging = true;
        ghostPos = { ...coords };
        return;
      }
    }

	// 3. Check if clicking their robots
	for (let i = 0; i < worldState.their_robots.length; i++) {
		const bot = worldState.their_robots[i];
		if (!bot.visible) continue;
		const dist = Math.hypot(coords.x - mToMM(bot.pose.position.x), coords.y - mToMM(bot.pose.position.y));
		if (dist < CLICK_THRESHOLD) {
			dragTarget = { type: 'their-robot', id: i };
			isDragging = true;
			ghostPos = { ...coords };
			return;
		}
	}
  }

  function handleMouseMove(e: MouseEvent) {
    if (!isDragging) return;
    const coords = getSVGCoords(e);
    ghostPos = { x: coords.x, y: coords.y };
  }

  function handleMouseUp(e: MouseEvent) {
    if (!isDragging || !dragTarget) return;
    
    const coords = getSVGCoords(e);
	teleport(dragTarget.type, dragTarget.id, coords.x, coords.y);

    isDragging = false;
    dragTarget = null;
  }
</script>

<div class="relative w-full h-full bg-zinc-950 rounded-xl overflow-hidden border border-zinc-800 shadow-2xl">
  
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <svg 
    bind:this={svgElement}
    {viewBox} 
    on:mousedown={handleMouseDown}
    on:mousemove={handleMouseMove}
    on:mouseup={handleMouseUp}
    on:mouseleave={() => { isDragging = false; dragTarget = null; }}
    class="w-full h-full cursor-crosshair select-none"
  >
	<!-- Field Lines -->
    <rect x={-mToMM(fieldDimensions.floor_length)/2} y={-mToMM(fieldDimensions.floor_width)/2} width={mToMM(fieldDimensions.floor_length)} height={mToMM(fieldDimensions.floor_width)} fill="#052e16" />
    <g fill="none" stroke="white" stroke-width={mToMM(fieldDimensions.line_width)} opacity="0.4">
      <rect x={-mToMM(fieldDimensions.length)/2} y={-mToMM(fieldDimensions.width)/2} width={mToMM(fieldDimensions.length)} height={mToMM(fieldDimensions.width)} />
      <line x1="0" y1={-mToMM(fieldDimensions.width)/2} x2="0" y2={mToMM(fieldDimensions.width)/2} />
      <circle cx="0" cy="0" r={mToMM(fieldDimensions.center_radius)} />
	  <rect x={-mToMM(fieldDimensions.length)/2} y={-mToMM(fieldDimensions.penalty_long_dist/2)} width={mToMM(fieldDimensions.penalty_short_dist)} height={mToMM(fieldDimensions.penalty_long_dist)} />
	  <rect x={mToMM((fieldDimensions.length/2) - fieldDimensions.penalty_short_dist)} y={-mToMM(fieldDimensions.penalty_long_dist/2)} width={mToMM(fieldDimensions.penalty_short_dist)} height={mToMM(fieldDimensions.penalty_long_dist)} />
    </g>

	<!-- Left Goal -->
	<g fill="none" stroke={!gameSettings.defend_plus_x ? (teamColor.is_blue ? 'blue' : 'yellow') : (teamColor.is_blue ? 'yellow' : 'blue')} stroke-width={mToMM(fieldDimensions.line_width)} opacity="0.4">
		<line x1={-mToMM(fieldDimensions.length/2+fieldDimensions.goal_depth)} y1={-mToMM(fieldDimensions.goal_width/2)} x2={-mToMM(fieldDimensions.length/2+fieldDimensions.goal_depth)} y2={mToMM(fieldDimensions.goal_width/2)} />
		<line x1={-mToMM(fieldDimensions.length/2+fieldDimensions.goal_depth)} y1={-mToMM(fieldDimensions.goal_width/2)} x2={-mToMM(fieldDimensions.length/2)} y2={-mToMM(fieldDimensions.goal_width/2)} />
		<line x1={-mToMM(fieldDimensions.length/2+fieldDimensions.goal_depth)} y1={mToMM(fieldDimensions.goal_width/2)} x2={-mToMM(fieldDimensions.length/2)} y2={mToMM(fieldDimensions.goal_width/2)} />
    </g>

	<!-- Right Goal -->
	 <g fill="none" stroke={gameSettings.defend_plus_x ? (teamColor.is_blue ? 'blue' : 'yellow') : (teamColor.is_blue ? 'yellow' : 'blue')} stroke-width={mToMM(fieldDimensions.line_width)} opacity="0.4">
		<line x1={mToMM(fieldDimensions.length/2+fieldDimensions.goal_depth)} y1={-mToMM(fieldDimensions.goal_width/2)} x2={mToMM(fieldDimensions.length/2+fieldDimensions.goal_depth)} y2={mToMM(fieldDimensions.goal_width/2)} />
		<line x1={mToMM(fieldDimensions.length/2+fieldDimensions.goal_depth)} y1={-mToMM(fieldDimensions.goal_width/2)} x2={mToMM(fieldDimensions.length/2)} y2={-mToMM(fieldDimensions.goal_width/2)} />
		<line x1={mToMM(fieldDimensions.length/2+fieldDimensions.goal_depth)} y1={mToMM(fieldDimensions.goal_width/2)} x2={mToMM(fieldDimensions.length/2)} y2={mToMM(fieldDimensions.goal_width/2)} />
    </g>

	<!-- Ghost Robot / Ball -->
    {#if isDragging && dragTarget}
      <g transform="translate({ghostPos.x}, {ghostPos.y})" class="pointer-events-none opacity-60">
        {#if dragTarget.type === 'ball'}
          <circle r={BALL_RADIUS} class="fill-orange-500/40 stroke-white stroke-[5px] stroke-dasharray-4" stroke-dasharray="20,10" />
        {:else}
          <circle r={ROBOT_RADIUS} class="{teamColor.is_blue ? 'fill-blue-500/30' : 'fill-yellow-500/30'} stroke-white stroke-[5px] stroke-dasharray-4" stroke-dasharray="40,20" />
          <text y="25" text-anchor="middle" font-size="80" font-weight="bold" fill="white">#{dragTarget.id}</text>
        {/if}
      </g>
    {/if}

	<!-- Ball -->
    {#if worldState.ball.visible}
      <circle cx={mToMM(worldState.ball.position.x)} cy={mToMM(worldState.ball.position.y)} r={BALL_RADIUS} class="fill-orange-500 stroke-white stroke-[10px]" />
    {/if}

	<!-- Our Robots -->
    {#each worldState.our_robots as bot, id}
      {#if bot.visible}
        <g transform="translate({mToMM(bot.pose.position.x)}, {mToMM(bot.pose.position.y)}) rotate({bot.pose.heading * 180 / Math.PI})">
          <circle r={ROBOT_RADIUS} class="{teamColor.is_blue ? 'fill-blue-600' : 'fill-yellow-500'} stroke-zinc-900 stroke-[15px]" />
		  <rect x="65" y="-35" width="25" height="70" fill="white" rx="4" class="drop-shadow-sm" />
          <text y="25" text-anchor="middle" font-size="80" font-weight="bold" fill="white">{id}</text>
        </g>
      {/if}
    {/each}

	<!-- Their Robots -->
    {#each worldState.their_robots as bot, id}
      {#if bot.visible}
        <g transform="translate({mToMM(bot.pose.position.x)}, {mToMM(bot.pose.position.y)}) rotate({bot.pose.heading * 180 / Math.PI})" opacity="0.7">
          <circle r={ROBOT_RADIUS} class="{!teamColor.is_blue ? 'fill-blue-600' : 'fill-yellow-500'} stroke-zinc-900 stroke-[15px]" />
		  <rect x="65" y="-35" width="25" height="70" fill="white" rx="4" class="drop-shadow-sm" />
          <text y="25" text-anchor="middle" font-size="80" font-weight="bold" fill="white">{id}</text>
        </g>
      {/if}
    {/each}
  </svg>
</div>