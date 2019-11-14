# Creating WAV files

Use the "say" command with the following options
* -v ? lists all voices
* -o output.wav
* required for .wav files : --data-format=LEF32@22050

say -v Samantha -o stone_detected.wav --data-format=LEF32@22050 "Skystone Detected"