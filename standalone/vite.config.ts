import { defineConfig } from "vite";
import viteReact from "@vitejs/plugin-react";
import tailwindcss from "@tailwindcss/vite";

export default defineConfig({
  root: __dirname,
  base: "./",
  plugins: [tailwindcss(), viteReact()],
  build: { outDir: "dist", emptyOutDir: true },
});
