{
  "assets": [
    {
      "file": "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items/Task3_Recon/recon.usd",
      "description": "Four open bins with 2 fire and 2 blood images. Visual only; approximate PVC layout.",
      "collision_count": 0,
      "rigid_body_count": 0,
      "sha256": "d4347c0d0b904ca34e88bb10a052478227eacbc230557809d15a19abd351c047"
    },
    {
      "file": "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items/Task4_Deploy/deploy.usd",
      "description": "0.61 m board, four true polygonal apertures (128 sides); visual only. Hole sizes/layout are explicit modeling assumptions.",
      "collision_count": 0,
      "rigid_body_count": 0,
      "sha256": "3a776e0df40b433baf89b031ff754e9bbf17ea789f0809bb2d4cab14db348651"
    },
    {
      "file": "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items/Task5_Octagon/octagon.usd",
      "description": "White horizontal octagonal PVC ring only; 2.7 m across flats; no signs/collisions/physics.",
      "collision_count": 0,
      "rigid_body_count": 0,
      "sha256": "3a7d5e45fa68fec7ff67d8c1ef44ed034b7e3d22ebd2e6ab6d8b93ab601db2da"
    },
    {
      "file": "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items/Task5_Table/restore_table.usd",
      "description": "Static collidable table and two open receiving trays. 0.61 m tabletop, height 0.75 m. No loose props.",
      "collision_count": 29,
      "rigid_body_count": 0,
      "sha256": "d7845053db835fca545a9ad6fed8bff54bd87a0e705302521462ed9f0c3d5379"
    },
    {
      "file": "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items/Task5_Objects/repair_bolt.usd",
      "description": "Dynamic rigid body, mass 0.2 kg, gravity enabled by default; bottom origin. 70 mm diameter x 50 mm",
      "collision_count": 2,
      "rigid_body_count": 1,
      "sha256": "cde8988dec76ef04df1e29a7cefac294ced352558baba4ed5e4d64562d53a905"
    },
    {
      "file": "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items/Task5_Objects/rescue_pill.usd",
      "description": "Dynamic rigid body, mass 0.2 kg, gravity enabled by default; bottom origin. 70 mm diameter x 50 mm",
      "collision_count": 2,
      "rigid_body_count": 1,
      "sha256": "0effd9fe823211777679f679a567162d62c6b911a2f182535a26a68b5004c77c"
    },
    {
      "file": "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items/Task5_Objects/repair_plug.usd",
      "description": "Dynamic rigid body, mass 0.2 kg, gravity enabled by default; bottom origin. 84 x 84 x 30 mm",
      "collision_count": 1,
      "rigid_body_count": 1,
      "sha256": "1397eeba497d7d7366a555ecc4b84060bb8983300fdcbbb3124e8c1ca76bd8b3"
    },
    {
      "file": "/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items/Task5_Objects/rescue_bandage.usd",
      "description": "Dynamic rigid body, mass 0.2 kg, gravity enabled by default; bottom origin. 84 x 84 x 30 mm",
      "collision_count": 1,
      "rigid_body_count": 1,
      "sha256": "e40fad6f7e721f4019f5a66cad3c11ae5cad534ac46b652edcfde1cbb6939949"
    }
  ],
  "coordinates": "Z up, metres. Props/table bottom at Z=0; octagon origin at ring centre. Panel faces -Y.",
  "physics_note": "Dynamic props fall only if containing scene gravity is nonzero. Existing pool was previously zero-gravity; this generator does not change it. No buoyancy/underwater drag authored.",
  "assumptions": "Recon support layout, tray dimensions, panel aperture diameters (0.14/0.096 m) and stand dimensions are adjustable visual approximations, not certified official CAD. Octagon is ring only as requested; hanging signs not included.",
  "texture_source": "Cropped illustrations supplied by user; previews may include source raster shading. Not official vector artwork."
}