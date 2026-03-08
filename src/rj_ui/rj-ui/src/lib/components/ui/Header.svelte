<script lang="ts">
  import { Button } from "$lib/components/ui/button";
  import { Separator } from "$lib/components/ui/separator";
  import { Octagon, Hand, Play, Trophy } from "lucide-svelte";

  import type { PlayState, MatchState, GameSettings } from "$lib/rosTypes";
  import { State, Restart, Period, periodToString } from "$lib/rosTypes";

  export let playState: PlayState;
  export let matchState: MatchState;
  export let gameSettings: GameSettings;

  // Ensure these are initialized to prevent "is not a function" errors
  export let onHalt: () => void = () => console.log("Halt triggered");
  export let onStop: () => void = () => console.log("Stop triggered");
  export let onStart: () => void = () => console.log("Start triggered");

  function stateStyles(state: State): string {
    if (state == State.Halt) {
        return "border-red-500/50 text-red-500 bg-red-500/10 shadow-[0_0_15px_rgba(239,68,68,0.2)]";
    } else if (state == State.Stop) {
        return "border-yellow-500/50 text-yellow-500 bg-yellow-500/10 shadow-[0_0_15px_rgba(234,179,8,0.2)]";
    } else {
        return "border-green-500/50 text-green-500 bg-green-500/10 shadow-[0_0_15px_rgba(34,197,94,0.2)]";
    }
  }
</script>

<header class="w-full bg-zinc-950 border-b border-zinc-800 p-4 flex items-center justify-between">
  
  <div class="flex items-center gap-2">
    <Button 
      variant='destructive' 
      size="sm"
      class="flex gap-2 font-bold uppercase tracking-wider transition-all"
      onclick={() => { console.log("Halt clicked"); onHalt(); }}
    >
      <Octagon size={16} fill={playState.state === State.Halt ? 'currentColor' : 'none'} />
      Halt
    </Button>

    <Button 
      variant='secondary' 
      size="sm"
      class="flex gap-2 font-bold uppercase tracking-wider bg-yellow-600 hover:bg-yellow-700 text-white"
      onclick={() => { console.log("Stop clicked"); onStop(); }}
    >
      <Hand size={16} fill={playState.state === State.Stop ? 'currentColor' : 'none'} />
      Stop
    </Button>

    <Button 
      variant='default' 
      size="sm"
      class="flex gap-2 font-bold uppercase tracking-wider bg-green-600 hover:bg-green-700 text-white"
      onclick={() => { console.log("Start clicked"); onStart(); }}
    >
      <Play size={16} fill={playState.state !== State.Halt && playState.state !== State.Stop ? 'currentColor' : 'none'} />
      Start
    </Button>
  </div>

  <div class="flex flex-col items-center">
    <span class="text-[10px] text-zinc-500 uppercase tracking-[0.2em] font-bold mb-1">Current Match State</span>
    <div class="px-8 py-1 rounded-full border-2 transition-all duration-500 font-mono text-xl font-black tracking-widest {stateStyles(playState.state)}">
      {State[playState.state]}
    </div>
  </div>

  <div class="flex items-center gap-4">
    <div class="flex flex-col items-center">
        <span class="text-2xl font-bold {gameSettings.simulation ? "text-green-500" : "text-zinc-500"}">SIM</span>
      </div>
    <div class="flex items-center bg-zinc-900 rounded-lg px-4 py-2 border border-zinc-800 gap-6">
      <div class="flex flex-col items-center">
        <span class="text-[10px] font-bold text-zinc-500 uppercase">Period</span>
        <span class="text-2xl font-bold text-zinc-500">{periodToString(matchState.period)}</span>
      </div>
    </div>
  </div>
</header>