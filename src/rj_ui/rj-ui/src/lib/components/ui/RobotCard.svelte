<script lang="ts">
  import { Card, CardContent, CardHeader, CardTitle } from "$lib/components/ui/card";
  import { Badge } from "$lib/components/ui/badge";
  import { Progress } from "$lib/components/ui/progress";
  import { Battery, Zap } from "lucide-svelte";

  import type { RobotStatus } from "$lib/rosTypes";
  import { KickerStatus } from "$lib/rosTypes";

  export let status: RobotStatus;
  export let robotImageUrl: string = "/robot-placeholder.png";

  // Helper to get kicker badge colors
  const kickerColorMap = {
    [KickerStatus.Unhealthy]: "bg-destructive text-destructive-foreground",
    [KickerStatus.Uncharged]: "bg-yellow-600 text-white",
    [KickerStatus.Charged]: "bg-green-600 text-white"
  };
</script>

<Card class="relative overflow-hidden transition-all duration-300 {status.alive ? 'border-primary shadow-lg ring-1 ring-primary/20' : 'opacity-60 grayscale border-muted'} bg-zinc-950 text-zinc-100 w-full max-w-xs">
  
  <CardHeader class="flex flex-row items-center justify-between space-y-0 pb-2">
    <div class="flex items-center gap-2">
      <div class="bg-primary text-primary-foreground font-mono font-bold px-2 py-1 rounded text-sm">
        ID: {status.robot_id.toString().padStart(2, '0')}
      </div>
      <CardTitle class="text-sm font-medium text-zinc-400 uppercase tracking-tighter">
        {status.position}
      </CardTitle>
    </div>
    <div class="h-2 w-2 rounded-full {status.alive ? 'bg-green-500 animate-pulse' : 'bg-zinc-700'}"></div>
  </CardHeader>

  <CardContent class="grid gap-4">
    <div class="relative flex justify-center py-4">
      <img 
        src={robotImageUrl} 
        alt="Robot {status.robot_id}" 
        class="h-32 w-auto object-contain transition-transform {status.has_ball_sense ? 'scale-105' : 'scale-100'}"
      />
      
      {#if status.has_ball_sense}
        <div class="absolute bottom-2 bg-orange-500 text-[10px] font-bold px-2 py-0.5 rounded-full uppercase tracking-widest animate-bounce shadow-[0_0_10px_rgba(249,115,22,0.6)]">
          Ball Sense
        </div>
      {/if}
    </div>

    <div class="grid grid-cols-2 gap-2 text-xs">
      <div class="flex flex-col gap-1">
        <span class="text-zinc-500 flex items-center gap-1"><Zap size={12}/> Kicker</span>
        <Badge class="w-fit text-[10px] uppercase {kickerColorMap[status.kicker_status]}">
          {KickerStatus[status.kicker_status]}
        </Badge>
      </div>

      <div class="flex flex-col gap-1">
        <span class="text-zinc-500 flex items-center gap-1"><Battery size={12}/> Battery</span>
        <div class="flex items-center gap-2">
          <Progress value={status.battery_percent} class="h-2 bg-zinc-800" />
          <span class="font-mono">{status.battery_percent}%</span>
        </div>
      </div>
    </div>
  </CardContent>
</Card>