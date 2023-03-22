// package frc.robot.commands.Autonomous;

// import edu.wpi.first.wpilibj2.command.Command;
// // import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.Drive.DriveDistanceCmd;
// import frc.robot.commands.Drive.DriveToPitch;
// import frc.robot.subsystem.DriveSubsystem;

// public final class Autos{

// public static SequentialCommandGroup balanceChargeStations(DriveSubsystem drive){
//     // return new InstantCommand(() -> {
//     //     new GetOnChargeStation(drive)
//     //     .withTimeout(AutoConstants.BalanceAuto.TIMEOUT_SECONDS)
//     //     .andThen(driveForDistance(drive, 
//     //     AutoConstants.BalanceAuto.DISTANCE_TO_CLOSER_CENTER,
//     //     AutoConstants.BalanceAuto.SPEED_TO_CLOSER_CENTER,
//     //     AutoConstants.BalanceAuto.REVERSED)
//     //     .withTimeout(AutoConstants.BalanceAuto.GET_CLOSER_TO_CENTER_TIMEOUT)).
//     //     andThen(new BalanceOnChargeStation(drive));
//     // });
//     var seqGroup = new SequentialCommandGroup(addCommands(new DriveToPitch(drive, .65, 10,false,false ));)
//    return 
//     // return new InstantCommand(() -> {
//     //   new DriveToPitch(drive, .65, 10,false,false )
//     //     // new GetOnChargeStation(drive)
//     //    // .withTimeout(AutoConstants.BalanceAuto.TIMEOUT_SECONDS)
//     //     // .andThen(new DriveDistanceCmd(drive,  AutoConstants.BalanceAuto.DISTANCE_TO_CLOSER_CENTER,.38 )
//     //     // .withTimeout(AutoConstants.BalanceAuto.GET_CLOSER_TO_CENTER_TIMEOUT))
//     //     //.andThen(new BalanceOnChargeStation(drive))
//     //     ;
//     // });



// }



// // public static Command driveForDistance(DriveSubsystem drive, double metersDistance, double speed,
// //       boolean isReversed) {
// //     final double speedDemand = speed * (isReversed ? -1 : 1);

// //     return new FunctionalCommand(() -> {
// //         drive.reset();
// //         drive.arcadeDrive(speedDemand,0);
// //     },
// //         () -> {
// //         },
// //         interrupted -> {
// //         },
// //         () -> {
// //           if (isReversed) {
// //             return drive.getLeftEncoderDistance() < -metersDistance
// //                 && drive.getRightEncoderDistance() < -metersDistance;
// //           }

// //           return drive.getLeftEncoderDistance() > metersDistance
// //               && drive.getRightEncoderDistance() > metersDistance;
// //         },
// //         drive);
// //   }

//   private Autos() {
//     throw new UnsupportedOperationException("This is a utility class!");
//   }

// }