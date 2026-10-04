#!/usr/bin/env node
/**
 * Verify example viewer pages exist and are valid.
 * Run: node scripts/verify/verify-example-viewers.mjs
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(__dirname, '..', '..');
const examplesDir = path.join(repoRoot, 'examples');

const EXPECTED_VIEWERS = [
  {
    html: '12_bamboo_mobile_robot_viewer.html',
    urdf: 'assets/icd/bamboo_mobile_robot.urdf',
    description: 'Example 12: ICD/LIS Bamboo Mobile Robot viewer'
  }
];

let failed = 0;

for (const viewer of EXPECTED_VIEWERS) {
  console.log(`\nChecking ${viewer.description}...`);
  
  // Check HTML exists
  const htmlPath = path.join(examplesDir, viewer.html);
  if (!fs.existsSync(htmlPath)) {
    console.error(`  ❌ HTML file not found: ${viewer.html}`);
    failed++;
    continue;
  }
  console.log(`  ✓ HTML file exists: ${viewer.html}`);
  
  // Check URDF exists
  const urdfPath = path.join(examplesDir, viewer.urdf);
  if (!fs.existsSync(urdfPath)) {
    console.error(`  ❌ URDF file not found: ${viewer.urdf}`);
    failed++;
    continue;
  }
  console.log(`  ✓ URDF file exists: ${viewer.urdf}`);
  
  // Validate HTML structure
  const htmlContent = fs.readFileSync(htmlPath, 'utf8');
  
  // Check for required elements
  const requiredElements = [
    '<!DOCTYPE html>',
    '<title>',
    'id="viewer"',
    'id="scrubber"',
    'three.js'
  ];
  
  for (const element of requiredElements) {
    if (!htmlContent.includes(element)) {
      console.error(`  ❌ HTML missing required element: ${element}`);
      failed++;
    }
  }
  
  // Check for joint state references
  const requiredData = [
    'START_JOINTS',
    'GOAL_JOINTS',
    'JOINT_NAMES'
  ];
  
  for (const data of requiredData) {
    if (!htmlContent.includes(data)) {
      console.error(`  ❌ HTML missing required data: ${data}`);
      failed++;
    }
  }
  
  console.log(`  ✓ HTML structure validated`);
  
  // Check file size (should be reasonable, not empty or huge)
  const stats = fs.statSync(htmlPath);
  if (stats.size < 1000) {
    console.error(`  ❌ HTML file too small (${stats.size} bytes), may be empty`);
    failed++;
  } else if (stats.size > 1000000) {
    console.error(`  ❌ HTML file too large (${stats.size} bytes), may have embedded assets`);
    failed++;
  } else {
    console.log(`  ✓ HTML file size OK (${stats.size} bytes)`);
  }
}

if (failed > 0) {
  console.error(`\n❌ ${failed} viewer validation check(s) failed`);
  process.exit(1);
} else {
  console.log(`\n✅ All ${EXPECTED_VIEWERS.length} viewer(s) validated successfully`);
}
