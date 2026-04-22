# Plan: Band Tabs — retina-spectrum feat/radio-sweep

## Context

The current UI shows FM (88–108), VHF Hi (174–216), and UHF (470–608) all on a single
40–700 MHz x-axis. FM channels at 200 kHz spacing are invisible at this scale. Adding
band tabs lets us zoom the FM band for peak-detection verification without losing the
TV band overview.

---

## Two tabs: FM | VHF/UHF

Frontend-only change (`web/index.html`). No backend or SSE protocol changes.
SSE data for all bands still arrives regardless of active tab.

### FM tab

- x-axis: **87.5 → 108.5 MHz** — 21 MHz range (~31× zoom vs current 40–700 view)
- Show: FM avg dataset (0) + pilot dots (3)
- Hide: VHF avg (1), UHF avg (2)
- Channel overlay: frequency comb — dashed boundary line at every FM channel lower edge
  (200 kHz grid). Labels at every 1 MHz (every 5th channel) — per-channel labels overlap
  at this zoom; 1 MHz grid is readable. At 1200px width, each channel ≈ 12px.
- Pilot dots make detected stations immediately obvious against the frequency comb.
- Per-step 3 MHz zoom not added — focus mode (click-to-zoom, 7.8 kHz/bin) handles that.
  Full 20 MHz FM view is better for detection verification (all stations visible at once).

### VHF/UHF tab (default on load)

- x-axis: **170 → 615 MHz** (tighter than current 40–700 — VHF starts at 174)
- Show: VHF avg (1), UHF avg (2), pilot dots (3)
- Hide: FM avg (0)
- Channel overlay: current TV channel boundaries + numbers (unchanged)

---

## Implementation

### Tab state

```js
let activeTab = 'vhf_uhf';  // or 'fm'
```

### Header markup

```html
<div id="tabs">
  <button id="tab-fm"      onclick="switchTab('fm')">FM</button>
  <button id="tab-vhf_uhf" onclick="switchTab('vhf_uhf')">VHF/UHF</button>
</div>
```

Minimal styling: active tab has a bottom border / brighter colour; inactive is muted.

### `switchTab(tab)` function

```js
function switchTab(tab) {
  activeTab = tab;
  if (tab === 'fm') {
    chart.options.scales.x.min = 87.5;
    chart.options.scales.x.max = 108.5;
    chart.data.datasets[0].hidden = false;  // FM avg
    chart.data.datasets[1].hidden = true;   // VHF avg
    chart.data.datasets[2].hidden = true;   // UHF avg
  } else {
    chart.options.scales.x.min = 170;
    chart.options.scales.x.max = 615;
    chart.data.datasets[0].hidden = true;
    chart.data.datasets[1].hidden = false;
    chart.data.datasets[2].hidden = false;
  }
  // pilot dots always visible
  updateTabButtons();
  chart.update('none');
}
```

Tab switching does **not** clear or re-request data — the chart data is always complete.

### Channel overlay changes

In `channelOverlayPlugin.afterDraw`, replace the hardcoded `channelsToShow` logic:

```js
// was: isNarrow ? ALL_CHANNELS : [...VHF_HI_CH, ...UHF_CH]
const channelsToShow = isNarrow
  ? ALL_CHANNELS
  : activeTab === 'fm'
    ? FM_CH
    : [...VHF_HI_CH, ...UHF_CH];
```

FM label throttle: for FM channels in FM tab view, skip the text label unless the
channel index is divisible by 5 (i.e., show only at 88.1, 89.1, 90.1 … every 1 MHz):

```js
if (ch.band === 'fm' && activeTab === 'fm') {
  // show label only every 5th channel (1 MHz grid) OR if chWidthPx > 20
  const idx = Math.round((ch.fc - 88.1) / 0.2);
  if (idx % 5 !== 0 && chWidthPx <= 20) skipLabel = true;
}
```

### `setAxisSweep()` → `setAxisForTab()`

Currently `setAxisSweep()` hardcodes `min:40, max:700`.
Replace with a call to `switchTab(activeTab)` — or just inline the tab logic —
so that a mode change back from focus restores the correct tab range, not the
old full-range view.

---

## Files changed

| File | Change |
|------|--------|
| `web/index.html` | Add tab buttons + CSS; add `switchTab()`; update `channelOverlayPlugin`; replace `setAxisSweep()` with tab-aware version |

---

## Verification

1. Load page → VHF/UHF tab active, x-axis 170–615 MHz, TV channels visible
2. Click FM tab → x-axis zooms to 87.5–108.5 MHz, FM freq comb visible, VHF/UHF lines hidden
3. FM stations detected (`is_pilot:true`) show green dots on FM tab at correct frequencies
4. Click back to VHF/UHF → TV view restored, FM dots still tracked internally
5. Enter focus mode from either tab → focus axis overrides tab range; back button restores
   correct tab range
