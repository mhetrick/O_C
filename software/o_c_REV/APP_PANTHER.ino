// Copyright (c) 2016 Michael Hetrick
//
// Author: Michael Hetrick (michael [dot] s [dot] hetrick [at] gmail [dot] com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// JAG-like cartesian CV plane.

#include "OC_apps.h"
#include "OC_menus.h"
#include "OC_strings.h"
#include "util/util_settings.h"

enum PANTHER_SETTINGS
{
  PANTHER_SETTING_OUTSOURCE,
  PANTHER_SETTING_OUT1SCALE,
  PANTHER_SETTING_OUT2SCALE,
  PANTHER_SETTING_OUT3SCALE,
  PANTHER_SETTING_OUT4SCALE,
  //PANTHER_SETTING_IN1RANGE,
  //PANTHER_SETTING_IN2RANGE
  PANTHER_NUM_SETTINGS
};

class PantherApp : public settings::SettingsBase<PantherApp, PANTHER_NUM_SETTINGS>
{
public:
  void Init()
  {
    InitDefaults();
    out1 = 0;
    out2 = 0;
    out3 = 0;
    out4 = 0;
    currentX = 0.0f;
    currentY = 0.0f;
    cvXIsBipolar = false;
    cvYIsBipolar = false;
    scaledCV3Output = 0;
  }

  uint8_t getOutputSource() const
  {
    return values_[PANTHER_SETTING_OUTSOURCE];
  }

  void ISR()
  {
    cv_posx.push(OC::ADC::smoothed_raw_value(ADC_CHANNEL_1));
    cv_posy.push(OC::ADC::smoothed_raw_value(ADC_CHANNEL_2));
    cv3.push(OC::ADC::smoothed_raw_value(ADC_CHANNEL_3));
    cv4.push(OC::ADC::value<ADC_CHANNEL_4>());

    currentX = cv_posx.value();
    currentY = cv_posy.value();
    currentXVisual = currentX >> 7;
    currentYVisual = currentY >> 7;

    scaledCV3Output = cv3.value() * 16;

    out1Factor = calcDistance(4096, 0);
    out2Factor = calcDistance(0, 0);
    out3Factor = calcDistance(4096, 4096);
    out4Factor = calcDistance(0, 4096);

    out1 = uint32_t(out1Factor * 16);
    out2 = uint32_t(out2Factor * 16);
    out3 = uint32_t(out3Factor * 16);
    out4 = uint32_t(out4Factor * 16);

    OC::DAC::set<DAC_CHANNEL_A>(out1);
    OC::DAC::set<DAC_CHANNEL_B>(out2);
    OC::DAC::set<DAC_CHANNEL_C>(out3);
    OC::DAC::set<DAC_CHANNEL_D>(out4);
  }

  uint32_t getScalingFactor() const
  {
    //if (getOutputSource()) return scaledCV3Output;
    return OC::DAC::MAX_VALUE;
  }

  //http://www.stm32duino.com/viewtopic.php?t=56
  uint32_t sqrt32(unsigned long n)
  {
    unsigned int c = 0x8000;
    unsigned int g = 0x8000;

    for(;;)
    {
        if(g*g > n)
        {
            g ^= c;
        }
        c >>= 1;
        if(c == 0)
        {
            return g;
        }
        g |= c;
    }
  }

  uint32_t calcDistance(int32_t outX, int32_t outY)
  {
      const int32_t xDiff = outX - currentX;
      const int32_t yDiff = outY - currentY;
      const uint32_t summed = xDiff*xDiff + yDiff*yDiff;
      return sqrt32(summed);
  }

  void RenderScreensaver() const;

  // ISR update is at 16.666kHz, we don't need it that fast so smooth the values to ~1Khz
  static constexpr int32_t kSmoothing = 16;
  SmoothedValue<uint32_t, kSmoothing> cv_posx;
  SmoothedValue<uint32_t, kSmoothing> cv_posy;
  SmoothedValue<uint32_t, kSmoothing> cv3;
  SmoothedValue<int32_t, kSmoothing> cv4;

private:
  uint32_t out1, out2, out3, out4;
  uint32_t out1Factor, out2Factor, out3Factor, out4Factor;
  uint32_t currentX, currentY;
  uint32_t currentXVisual, currentYVisual;
  uint32_t scaledCV3Output;
  bool cvXIsBipolar, cvYIsBipolar;
};


const char* const pantherOutSources[] =
{
  "+V", "CV3"
};


SETTINGS_DECLARE(PantherApp, PANTHER_NUM_SETTINGS)
{
  { 0, 0, 1, "Out Source", pantherOutSources, settings::STORAGE_TYPE_U8 },
  { 255, 0, 255, "Out 1", nullptr, settings::STORAGE_TYPE_U8 },
  { 255, 0, 255, "Out 2", nullptr, settings::STORAGE_TYPE_U8 },
  { 255, 0, 255, "Out 3", nullptr, settings::STORAGE_TYPE_U8 },
  { 255, 0, 255, "Out 4", nullptr, settings::STORAGE_TYPE_U8 },
};

PantherApp pantherApp;
struct{
  menu::ScreenCursor<menu::kScreenLines> cursor;
} pantherState;

// App stubs
void PANTHER_init()
{
  pantherState.cursor.Init(0, PANTHER_NUM_SETTINGS - 1);
  pantherApp.Init();
}

size_t PANTHER_storageSize()
{
  return PantherApp::storageSize();
}

size_t PANTHER_save(void *storage)
{
  return pantherApp.Save(storage);
}

size_t PANTHER_restore(const void *storage)
{
  return pantherApp.Restore(storage);
}

void FASTRUN PANTHER_isr()
{
  pantherApp.ISR();
}

void PANTHER_handleAppEvent(OC::AppEvent event)
{
  switch (event)
  {
    case OC::APP_EVENT_RESUME:
      break;
    case OC::APP_EVENT_SUSPEND:
    case OC::APP_EVENT_SCREENSAVER_ON:
    case OC::APP_EVENT_SCREENSAVER_OFF:
      break;
  }
}

void PANTHER_loop()
{

}

void PANTHER_menu()
{
  menu::DefaultTitleBar::Draw();

  menu::SettingsList<menu::kScreenLines, 0, menu::kDefaultValueX - 12> settings_list(pantherState.cursor);
  menu::SettingsListItem list_item;
  while (settings_list.available()) {
    const int current = settings_list.Next(list_item);
    list_item.DrawDefault(pantherApp.get_value(current), PantherApp::value_attr(current));
  }
}

void PantherApp::RenderScreensaver() const
{
  const uint16_t squareSize = 5;
  const uint16_t frameSize = 32;

  const uint16_t xPos = currentXVisual + 64;
  const uint16_t yPos = currentYVisual + 16;
  graphics.drawFrame(64, 16, frameSize, frameSize);
  graphics.drawRect(xPos, yPos, squareSize, squareSize);
}

void PANTHER_screensaver()
{
  OC::scope_render();
  pantherApp.RenderScreensaver();
}

void PANTHER_topButton()
{

}

void PANTHER_lowerButton()
{

}

void PANTHER_rightButton()
{
  pantherState.cursor.toggle_editing();
}

void PANTHER_leftButton()
{

}

void PANTHER_handleButtonEvent(const UI::Event &event)
{
  if (UI::EVENT_BUTTON_PRESS == event.type)
  {
    switch (event.control)
    {
      case OC::CONTROL_BUTTON_UP:
        PANTHER_topButton();
        break;
      case OC::CONTROL_BUTTON_DOWN:
        PANTHER_lowerButton();
        break;
      case OC::CONTROL_BUTTON_L:
        PANTHER_leftButton();
        break;
      case OC::CONTROL_BUTTON_R:
        PANTHER_rightButton();
        break;
    }
  }
}

void PANTHER_handleEncoderEvent(const UI::Event &event)
{
  if (OC::CONTROL_ENCODER_L == event.control)
  {

  }
  else if (OC::CONTROL_ENCODER_R == event.control)
  {
    if (pantherState.cursor.editing())
    {
      pantherApp.change_value(pantherState.cursor.cursor_pos(), event.value);
    }
    else
    {
      pantherState.cursor.Scroll(event.value);
    }
  }
}
