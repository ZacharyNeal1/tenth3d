﻿struct VS_IN
{
    float4 pos : POSITION;
    float4 col : COLOR;
    float4 tcd : TEXCOORD;
};

struct PS_IN
{
	float4 pos : SV_POSITION;
    float4 col : COLOR;
    float4 tcd : TEXCOORD;
};



Texture2D g_texture : register(t0);
SamplerState g_sampler : register(s0)
{
    Filter = MIN_MAG_MIP_LINEAR;
    AddressU = Wrap;
    AddressV = Wrap;
};

float4x4 worldViewProj;
const float PI = 3.1415926535;

PS_IN VS(VS_IN input)
{
	PS_IN output = (PS_IN) 0;
   
    output.pos = mul(input.pos, worldViewProj);
    output.col = input.col; //float4(1-clamp((output.pos.z / 400.0), 0.0, 1.0), 0.2, 0.2, 1.0);
    output.tcd = input.tcd;
    
    
    //output.pos = output.pos +0.2;
	return output;
}

float4 PS(PS_IN input) : SV_Target
{
    //float aperture = 178.0;
    //float apertureHalf = 0.5 * aperture * (PI / 180.0);
    //float maxFactor = sin(apertureHalf);
    
    //float2 uv;
    //float2 xy = 2.0 * input.tcd.xy - 1.0;
    //float d = length(xy);
    //if (d < (2.0 - maxFactor))
    //{
    //    d = length(xy * maxFactor);
    //    float z = sqrt(1.0 - d * d);
    //    float r = atan2(d, z) / PI;
    //    float phi = atan2(xy.y, xy.x);
    
    //    uv.x = r * cos(phi) + 0.5;
    //    uv.y = r * sin(phi) + 0.5;
    //}
    //else
    //{
    //    uv = input.tcd.xy;
    //}
    
    //return float4(g_texture.Sample(g_sampler, float2(input.tcd.x, input.tcd.y)).xyz, 1.0);
    if (input.tcd.z > 0.99)
        return lerp(float4(g_texture.Sample(g_sampler, float2(input.tcd.x, input.tcd.y)).xyz, 1.0), input.col, 0.5);
    else
        return input.col;


        //lerp(g_texture.Sample(g_sampler, float2(input.tcd.x, input.tcd.y)), input.col, 0.5);
        //return float4(1.0f, 1.0f, 1.0f, 1.0f);
}

