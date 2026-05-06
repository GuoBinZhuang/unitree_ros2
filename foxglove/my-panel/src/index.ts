import { ExtensionContext } from "@foxglove/extension";
import { initG1HeatmapPanel } from "./G1HeatmapPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "G1 关节热力图",
    initPanel: initG1HeatmapPanel,
  });
}