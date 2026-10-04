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
  
  // Check for required elements (flexible for different viewer versions)
  const requiredElements = [
    '<!DOCTYPE html>',
    '<title>'
  ];
  
  for (const element of requiredElements) {
    if (!htmlContent.includes(element)) {
      console.error(`  ❌ HTML missing required element: ${element}`);
      failed++;
    }
  }
  
  // Check for viewer container (either id="viewer" or id="view")
  if (!htmlContent.includes('id="viewer"') && !htmlContent.includes('id="view"')) {
    console.error(`  ❌ HTML missing viewer container (id="viewer" or id="view")`);
    failed++;
  }
  
  // Check for scrubber (either id="scrubber" or id="scrub")
  if (!htmlContent.includes('id="scrubber"') && !htmlContent.includes('id="scrub"')) {
    console.error(`  ❌ HTML missing scrubber control (id="scrubber" or id="scrub")`);
    failed++;
  }
  
  // Check for Three.js (CDN or local)
  if (!htmlContent.includes('three.js') && !htmlContent.includes('three.r128.min.js')) {
    console.error(`  ❌ HTML missing Three.js library reference`);
    failed++;
  }
  
  // If using local Three.js, verify the file exists
  if (htmlContent.includes('three.r128.min.js')) {
    const threeJsPath = path.join(examplesDir, 'three.r128.min.js');
    if (!fs.existsSync(threeJsPath)) {
      console.error(`  ❌ Local Three.js file not found: three.r128.min.js`);
      failed++;
    } else {
      console.log(`  ✓ Local Three.js library found`);
    }
  }
  
  // Check for joint state references (flexible for different formats)
  // Either START_JOINTS/GOAL_JOINTS or waypoint array (WP) with joint data or tasks array
  const hasOldFormat = htmlContent.includes('START_JOINTS') && 
                       htmlContent.includes('GOAL_JOINTS') && 
                       htmlContent.includes('JOINT_NAMES');
  const hasWaypointFormat = htmlContent.includes('var WP') && htmlContent.includes('label:');
  const hasTaskFormat = htmlContent.includes('var tasks') && 
                       (htmlContent.includes('firstTask') || htmlContent.includes('identity'));
  
  if (!hasOldFormat && !hasWaypointFormat && !hasTaskFormat) {
    console.error(`  ❌ HTML missing motion data (needs START_JOINTS/GOAL_JOINTS, WP array, or tasks array)`);
    failed++;
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
