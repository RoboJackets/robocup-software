<script>/**
 * @license lucide-svelte v0.562.0 - ISC
 *
 * ISC License
 *
 * Copyright (c) for portions of Lucide are held by Cole Bemis 2013-2023 as part of Feather (MIT). All other copyright (c) for Lucide are held by Lucide Contributors 2025.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * ---
 *
 * The MIT License (MIT) (for portions derived from Feather)
 *
 * Copyright (c) 2013-2023 Cole Bemis
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
import Icon from '../Icon.svelte';
const iconNode = [["path", { "d": "M10 10 7 7" }], ["path", { "d": "m10 14-3 3" }], ["path", { "d": "m14 10 3-3" }], ["path", { "d": "m14 14 3 3" }], ["path", { "d": "M14.205 4.139a4 4 0 1 1 5.439 5.863" }], ["path", { "d": "M19.637 14a4 4 0 1 1-5.432 5.868" }], ["path", { "d": "M4.367 10a4 4 0 1 1 5.438-5.862" }], ["path", { "d": "M9.795 19.862a4 4 0 1 1-5.429-5.873" }], ["rect", { "x": "10", "y": "8", "width": "4", "height": "8", "rx": "1" }]];
/**
 * @component @name Drone
 * @description Lucide SVG icon component, renders SVG Element with children.
 *
 * @preview ![img](data:image/svg+xml;base64,PHN2ZyAgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIgogIHdpZHRoPSIyNCIKICBoZWlnaHQ9IjI0IgogIHZpZXdCb3g9IjAgMCAyNCAyNCIKICBmaWxsPSJub25lIgogIHN0cm9rZT0iIzAwMCIgc3R5bGU9ImJhY2tncm91bmQtY29sb3I6ICNmZmY7IGJvcmRlci1yYWRpdXM6IDJweCIKICBzdHJva2Utd2lkdGg9IjIiCiAgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIgogIHN0cm9rZS1saW5lam9pbj0icm91bmQiCj4KICA8cGF0aCBkPSJNMTAgMTAgNyA3IiAvPgogIDxwYXRoIGQ9Im0xMCAxNC0zIDMiIC8+CiAgPHBhdGggZD0ibTE0IDEwIDMtMyIgLz4KICA8cGF0aCBkPSJtMTQgMTQgMyAzIiAvPgogIDxwYXRoIGQ9Ik0xNC4yMDUgNC4xMzlhNCA0IDAgMSAxIDUuNDM5IDUuODYzIiAvPgogIDxwYXRoIGQ9Ik0xOS42MzcgMTRhNCA0IDAgMSAxLTUuNDMyIDUuODY4IiAvPgogIDxwYXRoIGQ9Ik00LjM2NyAxMGE0IDQgMCAxIDEgNS40MzgtNS44NjIiIC8+CiAgPHBhdGggZD0iTTkuNzk1IDE5Ljg2MmE0IDQgMCAxIDEtNS40MjktNS44NzMiIC8+CiAgPHJlY3QgeD0iMTAiIHk9IjgiIHdpZHRoPSI0IiBoZWlnaHQ9IjgiIHJ4PSIxIiAvPgo8L3N2Zz4K) - https://lucide.dev/icons/drone
 * @see https://lucide.dev/guide/packages/lucide-svelte - Documentation
 *
 * @param {Object} props - Lucide icons props and any valid SVG attribute
 * @returns {FunctionalComponent} Svelte component
 *
 */
</script>

<Icon name="drone" {...$$props} iconNode={iconNode}>
  <slot/>
</Icon>
