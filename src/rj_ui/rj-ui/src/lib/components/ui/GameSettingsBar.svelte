<script lang="ts">
  import { Button } from "$lib/components/ui/button";
  import { Separator } from "$lib/components/ui/separator";
  import type { GameSettings, GoalieMsg, TeamColorMsg } from "$lib/rosTypes";
  import { Shield, Users, Map, Power } from "lucide-svelte";

  export let teamColor: TeamColorMsg;
  export let goalieId: GoalieMsg;
  export let gameSettings: GameSettings;

  export let onSettingsChange: (settings: GameSettings) => void = () => {};

  // Local handler to ensure the value is treated as a number
  function handleGoalieInput(e: Event) {
    const target = e.target as HTMLInputElement;
    const val = parseInt(target.value);
    if (!isNaN(val)) {
      let newSettings = gameSettings;
      newSettings.request_goalie_id = val;
      onSettingsChange(newSettings);
    }
  }

  function toggleTeamColor() {
    let newSettings = gameSettings;
    newSettings.request_blue_team = !teamColor.is_blue;
    onSettingsChange(newSettings);
  }

  function toggleDefendPlusX() {
    let newSettings = gameSettings;
    newSettings.defend_plus_x = !gameSettings.defend_plus_x;
    onSettingsChange(newSettings);
  }

  function toggleUseOurHalf() {
    let newSettings = gameSettings;
    newSettings.use_our_half = !gameSettings.use_our_half;
    onSettingsChange(newSettings);
  }

  function toggleUseTheirHalf() {
    let newSettings = gameSettings;
    newSettings.use_their_half = !gameSettings.use_their_half;
    onSettingsChange(newSettings);
  }
</script>

<div class="w-full bg-zinc-900/90 border-b border-zinc-800 px-4 py-1.5 flex items-center justify-between backdrop-blur-sm">
  
  <div class="flex items-center gap-5">
    <div class="flex items-center gap-2">
      <Users size={14} class="text-zinc-400" />
      <span class="text-[9px] font-bold uppercase text-zinc-500 tracking-wider">Team</span>
      <button 
        onclick={() => toggleTeamColor()}
        class="flex items-center gap-2 px-2 py-0.5 rounded border border-zinc-700 bg-zinc-950 hover:bg-zinc-800 transition-colors"
      >
        <div class="w-1.5 h-1.5 rounded-full {teamColor.is_blue ? 'bg-blue-500 shadow-[0_0_5px_rgba(59,130,246,0.5)]' : 'bg-yellow-500 shadow-[0_0_5px_rgba(234,179,8,0.5)]'}"></div>
        <span class="text-xs font-mono uppercase text-zinc-400 font-bold">{teamColor.is_blue ? "Blue" : "Yellow"}</span>
      </button>
    </div>

    <Separator orientation="vertical" class="h-4 bg-zinc-800" />

    <div class="flex items-center gap-2">
      <Shield size={14} class="text-zinc-400" />
      <span class="text-[9px] font-bold uppercase text-zinc-500 tracking-wider">Goalie ID</span>
      <input 
        type="number" 
        min="0" 
        max="15"
        value={goalieId.goalie_id}
        oninput={handleGoalieInput}
        class="w-12 bg-zinc-950 border border-zinc-700 text-xs text-zinc-400 font-mono rounded px-1.5 py-0.5 text-center focus:outline-none focus:border-blue-500 focus:ring-1 focus:ring-blue-500/20"
      />
    </div>

    <Separator orientation="vertical" class="h-4 bg-zinc-800" />

    <div class="flex items-center gap-2">
      <Map size={14} class="text-zinc-400" />
      <span class="text-[9px] font-bold uppercase text-zinc-500 tracking-wider">Defending</span>
      <button 
        onclick={() => toggleDefendPlusX()}
        class="text-xs font-mono px-2 py-0.5 rounded border border-zinc-700 bg-zinc-950 text-zinc-400 hover:bg-zinc-800"
      >
        {gameSettings.defend_plus_x ? '+X Axis' : '-X Axis'}
      </button>
    </div>
  </div>

  <div class="flex items-center gap-4">
    <Button 
      variant="secondary"
      class="h-7 px-3 text-[10px] font-bold uppercase tracking-tight bg-zinc-800 hover:bg-zinc-700 border border-zinc-600"
      onclick={toggleUseOurHalf}
    >
      <Power size={11} class="mr-1.5 {gameSettings.use_our_half ? 'text-blue-400' : 'text-zinc-400'}" />
      <span class="px-2 py-0.5 text-zinc-400">Toggle Our Half</span>
    </Button>
    
    <Button 
      variant="secondary"
      class="h-7 px-3 text-[10px] font-bold uppercase tracking-tight bg-zinc-800 hover:bg-zinc-700 border border-zinc-600"
      onclick={toggleUseTheirHalf}
    >
      <Power size={11} class="mr-1.5 {gameSettings.use_their_half ? 'text-blue-400' : 'text-zinc-400'}" />
      <span class="px-2 py-0.5 text-zinc-400">Toggle Their Half</span>
    </Button>
  </div>
</div>