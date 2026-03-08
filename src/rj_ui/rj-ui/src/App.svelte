<script lang="ts">
	import './app.css';
	import { fly } from 'svelte/transition';
	import { onMount } from "svelte";
	import { init } from "@neutralinojs/lib";
	import { Toaster } from 'svelte-sonner';
	import { resizable } from '$lib/actions/resizable';
	import { PanelRightClose, PanelRightOpen } from 'lucide-svelte';

	import Button from '$lib/components/ui/button/button.svelte';
	import RobotCard from '$lib/components/ui/RobotCard.svelte';
	import Header from '$lib/components/ui/Header.svelte';
	import GameSettingsBar from '$lib/components/ui/GameSettingsBar.svelte';
	import TeamSidebar from '$lib/components/ui/TeamSidebar.svelte';
	import Field from '$lib/components/Field.svelte';

	import { State } from "$lib/rosTypes";

	import {
		initROS,
		fieldDimensions,
		robotStatuses,
		worldState,
		playState,
		matchState,
		goalie,
		teamColor,
		ourTeamInfo,
		theirTeamInfo,
		gameSettings,
		controlCommands,
		teleportBall,
		teleportRobot,
		setGameSettings,
		setPlayState
	} from "$lib/stores/rosStores";

	function setState(state: State) {
		let newPlayState =$playState;
		newPlayState.state = state;
		setPlayState(newPlayState);
	}

	function teleport(type: 'our-robot' | 'their-robot' | 'ball', id: number | null | undefined, x: number, y: number) {
		if (type === 'ball') {
			teleportBall({ x: x / 1000, y: y / 1000});
		} else if (type === 'our-robot') {
			teleportRobot(
				$teamColor.is_blue,
				id ?? 0,
				{
					position: { x: x / 1000, y: y / 1000},
					heading: $worldState.our_robots[id ?? 0].pose.heading
				}
			);
		} else {
			teleportRobot(
				!$teamColor.is_blue,
				id ?? 0,
				{
					position: { x: x / 1000, y: y / 1000 },
					heading: $worldState.their_robots[id ?? 0].pose.heading
				}
			);
		}
	}

	let isSidebarOpen = true;

	onMount(async () => {
		init();

		initROS("ws://localhost:9090");
	});
</script>

<Toaster richColors position="bottom-right" />

<div class="flex flex-col h-screen w-full bg-black text-zinc-100 overflow-hidden">
	<Header
		playState={$playState}
		matchState={$matchState}
		gameSettings={$gameSettings}
		onHalt={() => setState(State.Halt)}
		onStop={() => setState(State.Stop)}
		onStart={() => setState(State.Playing)}
	/>
	<GameSettingsBar
		teamColor={$teamColor}
		goalieId={$goalie}
		gameSettings={$gameSettings}
		onSettingsChange={(settings) => setGameSettings(settings)}
	/>

	<main class="flex flex-1 overflow-hidden relative">
		<aside 
			use:resizable 
			class="relative min-w-[180px] max-w-[800px] border-r border-zinc-800 bg-zinc-900/50 flex flex-col"
			style="width: 320px;"
		>
			<div class="p-4 border-b border-zinc-800">
			<h2 class="text-xs font-bold uppercase tracking-widest text-zinc-500">Fleet</h2>
			</div>

			<div class="flex-1 overflow-y-auto p-3 grid gap-3 content-start"
				style="grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); grid-auto-rows: 340px">
			{#each $robotStatuses as status}
				<RobotCard status={status} />
			{/each}
			</div>
		</aside>

		<section class="flex-1 relative bg-zinc-900/20 p-6 flex items-center justify-center">
		<Field
			fieldDimensions={$fieldDimensions}
			worldState={$worldState}
			teamColor={$teamColor}
			gameSettings={$gameSettings}
			teleport={teleport}
		/>
		
		<div class="absolute top-4 right-4 z-20">
			<Button 
			variant="outline" 
			size="icon" 
			class="bg-zinc-900 border-zinc-700"
			onclick={() => isSidebarOpen = !isSidebarOpen}
			>
			{#if isSidebarOpen} <PanelRightClose size={18}/> {:else} <PanelRightOpen size={18}/> {/if}
			</Button>
		</div>
		</section>

		{#if isSidebarOpen}
		<div 
			transition:fly={{ x: 320, duration: 300 }}
			class="h-full z-10"
		>
			<TeamSidebar ourTeamInfo={$ourTeamInfo} theirTeamInfo={$theirTeamInfo} />
		</div>
		{/if}
	</main>
</div>
