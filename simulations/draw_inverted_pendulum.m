(* ::Package:: *)

drawInvertedPendulum[\[Theta]_, x_] := Module[
  {
    cartHeight = 1,
    cartWidth = 2,
    pendulumLength = 3,
    pendulumRadius = 0.25
  },
  Graphics[
    {
      (* Cart *)
      EdgeForm[{Black, Thick}],
      FaceForm[],
      Rectangle[
        {x - cartWidth / 2, 0},
        {x + cartWidth / 2, cartHeight
      }],

      (* Pendulum *)
      Line[{
        {x, cartHeight / 2},
        {x - pendulumLength Sin[\[Theta]], cartHeight / 2 + pendulumLength Cos[\[Theta]]}
      }],
      FaceForm[Black],
      Disk[
        {
          x - (pendulumLength + pendulumRadius) Sin[\[Theta]],
          cartHeight / 2 + (pendulumLength + pendulumRadius) Cos[\[Theta]]
        },
        pendulumRadius
      ]
    },
    Axes -> True,
    PlotRange -> {{-10, 10}, {-5, 5}}
  ]
];
