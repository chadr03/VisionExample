/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import easypath.EasyPath;
import easypath.EasyPathConfig;
import easypath.FollowPath;
import easypath.Path;
import easypath.PathUtil;


public class FollowCommandGroup extends CommandGroup {
  /**
   * Add your docs here.
   */
  public FollowCommandGroup() {
		addSequential(new FollowPath(
      new Path(t -> 
		/* {"start":{"x":13,"y":165},"mid1":{"x":46,"y":163},"mid2":{"x":4,"y":185},"end":{"x":62,"y":183}} */
		(-144 * Math.pow(t, 2) + 144 * t + -6) / (525 * Math.pow(t, 2) + -450 * t + 99),
		58.581), x -> {
          System.out.println(x);
          if (x < 0.15) return 0.45;
		          else if (x < 0.75) return 0.55;
		          else return 0.45;
            }));

    // addSequential(new FollowPath(        
    //     new Path(t -> 
    //     /* {"start":{"x":0,"y":165},"mid1":{"x":46,"y":163},"mid2":{"x":50,"y":145},"end":{"x":50,"y":79}} */
    //     (-96 * Math.pow(t, 2) + -96 * t + -6) / (114 * Math.pow(t, 2) + -252 * t + 138),
    //     116.77), x -> {
    //     System.out.println(x);
    //     if (x < 0.15) return 0.45;
    //           else if (x < 0.75) return 0.55;
    //           else return 0.45;
    //         }));
  }
}
