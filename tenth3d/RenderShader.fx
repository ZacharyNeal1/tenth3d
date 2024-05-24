struct VS_IN
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
    //return float4(1.0f, 1.0f, 1.0f, 1.0f);
    if (input.tcd.z == 1.0)
        return lerp(float4(g_texture.Sample(g_sampler, float2(input.tcd.x, input.tcd.y)).xyz, 1.0), input.col, 0.5); //lerp(g_texture.Sample(g_sampler, float2(input.tcd.x, input.tcd.y)), input.col, 0.5);
    else
        return input.col;

}

