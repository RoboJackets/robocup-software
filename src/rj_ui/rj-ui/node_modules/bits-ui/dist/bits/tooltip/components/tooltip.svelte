<script lang="ts">
	import { boxWith } from "svelte-toolbelt";
	import type { TooltipRootProps } from "../types.js";
	import { TooltipRootState } from "../tooltip.svelte.js";
	import FloatingLayer from "../../utilities/floating-layer/components/floating-layer.svelte";
	import { noop } from "../../../internal/noop.js";

	let {
		open = $bindable(false),
		onOpenChange = noop,
		onOpenChangeComplete = noop,
		disabled,
		delayDuration,
		disableCloseOnTriggerClick,
		disableHoverableContent,
		ignoreNonKeyboardFocus,
		children,
	}: TooltipRootProps = $props();

	TooltipRootState.create({
		open: boxWith(
			() => open,
			(v) => {
				open = v;
				onOpenChange(v);
			}
		),
		delayDuration: boxWith(() => delayDuration),
		disableCloseOnTriggerClick: boxWith(() => disableCloseOnTriggerClick),
		disableHoverableContent: boxWith(() => disableHoverableContent),
		ignoreNonKeyboardFocus: boxWith(() => ignoreNonKeyboardFocus),
		disabled: boxWith(() => disabled),
		onOpenChangeComplete: boxWith(() => onOpenChangeComplete),
	});
</script>

<FloatingLayer tooltip>
	{@render children?.()}
</FloatingLayer>
