<script lang="ts">
  import { Shield, Timer, AlertTriangle, Ban } from "lucide-svelte";
  import { Separator } from "$lib/components/ui/separator";
    import type { TeamInfo } from "$lib/rosTypes";

  export let ourTeamInfo: TeamInfo;
  export let theirTeamInfo: TeamInfo;
</script>

<aside class="w-64 h-full bg-zinc-950 border-l border-zinc-800 flex flex-col overflow-hidden">
  <div class="p-3 border-b border-zinc-800 bg-zinc-900/50">
    <h2 class="text-[10px] font-bold uppercase tracking-widest text-zinc-500 text-center">Match Overview</h2>
  </div>

  <div class="flex-1 flex flex-col p-4 gap-8 overflow-y-auto">
    
    <section class="flex flex-col gap-4">
      <div class="flex items-center gap-2">
        <div class="w-1.5 h-6 bg-blue-500 rounded-full"></div>
        <h3 class="text-lg font-black tracking-tight text-white uppercase truncate">{ourTeamInfo.name}</h3>
      </div>

      <div class="grid grid-cols-2 gap-2">
        <div class="col-span-2 bg-blue-500/10 border border-blue-500/30 rounded-lg p-4 flex flex-col items-center">
          <span class="text-[10px] font-bold text-blue-400 uppercase tracking-tighter mb-1">Score</span>
          <span class="text-5xl font-mono font-black text-white">{ourTeamInfo.score}</span>
        </div>

        <div class="bg-zinc-900 border border-zinc-800 rounded p-2 flex flex-col items-center gap-1">
          <div class="flex items-center gap-1 text-[9px] font-bold text-zinc-500 uppercase">
             <AlertTriangle size={10} class="text-yellow-500" /> Yellow
          </div>
          <span class="text-xl font-mono font-bold text-yellow-500">{ourTeamInfo.num_yellow_cards}</span>
        </div>

        <div class="bg-zinc-900 border border-zinc-800 rounded p-2 flex flex-col items-center gap-1">
          <div class="flex items-center gap-1 text-[9px] font-bold text-zinc-500 uppercase">
             <Ban size={10} class="text-red-500" /> Red
          </div>
          <span class="text-xl font-mono font-bold text-red-500">{ourTeamInfo.num_red_cards}</span>
        </div>

        <div class="flex items-center justify-between col-span-2 px-1 mt-2 text-[10px] font-mono">
          <div class="flex items-center gap-2 text-zinc-400">
            <Timer size={12} /> Timeouts
          </div>
          <span class="text-white font-bold">{ourTeamInfo.timeouts_left} Left</span>
        </div>

        <div class="flex items-center justify-between col-span-2 px-1 text-[10px] font-mono">
          <div class="flex items-center gap-2 text-zinc-400">
            <Shield size={12} /> Goalie ID
          </div>
          <span class="text-blue-400 font-bold">#{ourTeamInfo.goalie_id}</span>
        </div>
      </div>
    </section>

    <Separator class="bg-zinc-800" />

    <section class="flex flex-col gap-4">
      <div class="flex items-center gap-2">
        <div class="w-1.5 h-6 bg-yellow-500 rounded-full"></div>
        <h3 class="text-lg font-black tracking-tight text-white uppercase truncate">{theirTeamInfo.name}</h3>
      </div>

      <div class="grid grid-cols-2 gap-2">
        <div class="col-span-2 bg-yellow-500/10 border border-yellow-500/30 rounded-lg p-4 flex flex-col items-center">
          <span class="text-[10px] font-bold text-yellow-400 uppercase tracking-tighter mb-1">Score</span>
          <span class="text-5xl font-mono font-black text-white">{theirTeamInfo.score}</span>
        </div>

        <div class="bg-zinc-900 border border-zinc-800 rounded p-2 flex flex-col items-center gap-1">
          <div class="flex items-center gap-1 text-[9px] font-bold text-zinc-500 uppercase">
             <AlertTriangle size={10} class="text-yellow-500" /> Yellow
          </div>
          <span class="text-xl font-mono font-bold text-yellow-500">{theirTeamInfo.num_yellow_cards}</span>
        </div>

        <div class="bg-zinc-900 border border-zinc-800 rounded p-2 flex flex-col items-center gap-1">
          <div class="flex items-center gap-1 text-[9px] font-bold text-zinc-500 uppercase">
             <Ban size={10} class="text-red-500" /> Red
          </div>
          <span class="text-xl font-mono font-bold text-red-500">{theirTeamInfo.num_red_cards}</span>
        </div>

        <div class="flex items-center justify-between col-span-2 px-1 mt-2 text-[10px] font-mono">
          <div class="flex items-center gap-2 text-zinc-400">
            <Timer size={12} /> Timeouts
          </div>
          <span class="text-white font-bold">{theirTeamInfo.timeouts_left} Left</span>
        </div>

        <div class="flex items-center justify-between col-span-2 px-1 text-[10px] font-mono">
          <div class="flex items-center gap-2 text-zinc-400">
            <Shield size={12} /> Goalie ID
          </div>
          <span class="text-yellow-400 font-bold">#{theirTeamInfo.goalie_id}</span>
        </div>
      </div>
    </section>
  </div>
</aside>