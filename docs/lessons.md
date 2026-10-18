# Guided lessons

Seven lessons run real analyses on your movement and ask you to do something with the answer. They
use your project when you give one and a built-in demo movement otherwise, so you can start before
you have a CAD file.

```
ohs lesson list
ohs lesson show train --project movement.hsproj
ohs lesson check train --project movement.hsproj --answer 840
```

| Lesson | You learn | Exercise |
| --- | --- | --- |
| `train` | Follow the power, do the beat arithmetic, see where the torque goes | Work out the escape wheel's turns per hour (`--answer`) |
| `timegrapher` | What rate, amplitude and beat error mean; orientation and wind | Find the crown-up rate at half wind (`--answer`) |
| `hairspring` | Frequency from stiffness and inertia; regulating | A what-if study that makes the watch run 10 s/day fast (`--study`) |
| `mainspring` | Torque, amplitude and power reserve | A study that gains 8 degrees of amplitude without knocking (`--study`) |
| `escapement` | Where one beat's energy goes; drop, lift, unlocking | A study that gains 4 degrees without costing isochronism (`--study`) |
| `jewelling` | Press-fit interference and what it does to the plate | The hole diameter for a 2.0 mm jewel at 10 um (`--answer`) |
| `skeletonise` | Remove metal from a bridge under a 5000 g shock | A window size that cuts 20 % of the mass at safety factor 2 (`--cutout`) |

## How a lesson works

`ohs lesson show <id>` prints the explanation in steps, with the figures of *your* movement next to
each: ratios, torques, amplitudes, energies, pressures. Each step may suggest a command to try.
Then the exercise: do the work with the ordinary commands, and give the result to
`ohs lesson check`. The check runs the same analyses, says what passed and what did not, and tells
you what to change. It exits 0 when the exercise is done and 1 when it is not.

For the study exercises you write a what-if file, for example:

```toml
[study]
name = "regulate fast"

[[change]]
parameter = "hairspring_length_mm"
scale = 0.99977
```

```
ohs lesson check hairspring --study study.toml
```

The check also refuses changes outside the exercise (the hairspring lesson allows the hairspring
and the balance inertia only), so you learn the relation instead of fiddling with everything.

`ohs lesson check <id> --solution` prints a working answer if you get stuck.

## With OM10

Give your imported OM10 project to every command and the numbers become OM10's. The
[OM10 tutorial](om10.md) strings the lessons into a complete modification study.
