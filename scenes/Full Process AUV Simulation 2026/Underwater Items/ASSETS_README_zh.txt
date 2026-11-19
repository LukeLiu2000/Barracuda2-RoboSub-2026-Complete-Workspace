新增资产共8个USD，均为米制Z向上，使用defaultPrim可直接Reference。
Task3_Recon/recon.usd：四筐和管架，纯视觉。
Task4_Deploy/deploy.usd：四真孔鱼雷板，纯视觉，正面朝-Y。
Task5_Octagon/octagon.usd：白色八角环，纯视觉，原点在环中心。
Task5_Table/restore_table.usd：空桌+两个开口接收筐，静态碰撞；底面原点Z=0。
Task5_Objects/：repair_bolt.usd、rescue_pill.usd（圆罐），repair_plug.usd、rescue_bandage.usd（方盒）；每个0.2kg动态刚体，原点在底面。
保留textures相对目录。模型为依据示意图的简化重建，不是官方CAD；未明确的孔径、支架与筐尺寸是近似值。
下坠需要所引用场景具有非零重力；本次未更改pool重力、机器人、已有Gate/Slalom或场景引用。
预览task_assets_preview.png；详细清单task_assets_manifest.json；生成源码_task_asset_sources。

V2修订：鱼雷板已使用用户新发完整正面图，真实孔洞对齐原图红圈；Task5六个图标已重新生成高清明亮卡通贴图，圆罐使用圆形白底贴纸并保留留白。旧版本已备份，详情task_assets_revision_v2.json。重新打开对应USD或Fetch重载即可；不必退出Isaac。
