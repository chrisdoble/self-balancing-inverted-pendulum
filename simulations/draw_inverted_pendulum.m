(* ::Package:: *)

drawInvertedPendulum[l_, \[Theta]_, x_] := Module[
  {
    cartHeight = l / 2,
    cartWidth = l,
    pendulumRadius = l / 10
  },
  Magnify[
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
          {x - l Sin[\[Theta]], cartHeight / 2 + l Cos[\[Theta]]}
        }],
        FaceForm[Black],
        Disk[
          {
            x - (l + pendulumRadius) Sin[\[Theta]],
            cartHeight / 2 + (l + pendulumRadius) Cos[\[Theta]]
          },
          pendulumRadius
        ]
      },
      Axes -> True,
      PlotRange -> {{-10l, 10l}, {-l, 2l}}
    ],
    2
  ]
];
